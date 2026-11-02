function result = test_real_imu(dataFile, varargin)
%TEST_REAL_IMU 单 IMU 实测数据：六轴 AHRS、Raw INS 与 15 状态 ESKF + ZUPT。
% 在工程根目录运行（基础 MATLAB，R2018b 或更新版本）：
%   addpath('test');
%   result = test_real_imu;
%   result = test_real_imu('data/imu_100hz.txt');
%   result = test_real_imu('data/imu_500hz.txt');
%   result = test_real_imu([], 'ZUPTMode', 'none');
%   result = test_real_imu([], 'ShowFigures', false);
%
% 输入：step 0 qw qx qy qz ax ay az wx wy wz;T <整数纳秒时间戳>
% 比力 m/s^2，角速度 rad/s，四元数 wxyz。忽略 # 注释。
% 初始静止段默认 2 s，初始位置/速度为零；重力对准，初始 yaw=0。
% 按实际时间戳统一处理不同采样率，采用 IMU 静止检测辅助零速更新。
% 导航从加速度计/陀螺仪独立估计姿态，同时保留设备四元数供检查。
% 输出为局部坐标系下的估计位移，单位 m。

root = fileparts(fileparts(mfilename('fullpath')));
if nargin < 1 || isempty(dataFile)
    dataFile = fullfile(root, 'data', 'imu_100hz.txt');
end
p = inputParser;
p.addParameter('InitialStaticSeconds', 2, @positiveScalar);
p.addParameter('StationaryWindowSeconds', 0.15, @positiveScalar);
p.addParameter('MinimumStationarySeconds', 0.20, @positiveScalar);
p.addParameter('GyroThresholdRadS', 0.08, @positiveScalar);
p.addParameter('ForceToleranceMS2', 0.30, @positiveScalar);
p.addParameter('ZUPTMode', 'glrt', @(x) any(strcmpi(x, {'glrt','none'})));
p.addParameter('ShowFigures', true, @(x) islogical(x) && isscalar(x));
p.parse(varargin{:});
opt = p.Results;
cfg = jsondecode(fileread(fullfile(root, 'config.json')));
g = cfg.gravity_m_s2;
ahrs = cfg.robot.ahrs;
ekf = cfg.robot.eskf;
quality = cfg.robot.quality;
detector = cfg.robot.stationary;
% 100 Hz / 500 Hz 使用相同的时间窗口定义，按实际时间维护。
detector.window_s = opt.StationaryWindowSeconds;
detector.minimum_stationary_s = opt.MinimumStationarySeconds;
detector.gyro_limit_rad_s = opt.GyroThresholdRadS;
detector.force_tolerance_m_s2 = opt.ForceToleranceMS2;

D = readRecording(char(dataFile));
t = D.time_s;
n = numel(t);
assert(t(end) > opt.InitialStaticSeconds, 'RealIMU:CalibrationLength', ...
    '记录长度必须大于 InitialStaticSeconds。');
dt = diff(t);
medianDt = median(dt);
assert(detector.window_s >= 2 * medianDt, 'RealIMU:DetectorWindow', ...
    'StationaryWindowSeconds 至少需要覆盖两个采样间隔。');
if max(dt) > 5 * medianDt
    warning('RealIMU:TimeGap', ...
        '最大采样间隔 %.4f s；存在较大时间空缺。按实际 dt 积分，未插值。', max(dt));
end

cal = t <= opt.InitialStaticSeconds;
assert(nnz(cal) >= 10, 'RealIMU:CalibrationSamples', '初始静止段至少需要 10 帧。');
f0 = mean(D.force_S_m_s2(cal,:), 1)';
bg0 = mean(D.gyro_S_rad_s(cal,:), 1)';
forceStd = std(D.force_S_m_s2(cal,:), 0, 1);
gyroStd = std(D.gyro_S_rad_s(cal,:), 0, 1);
assert(abs(norm(f0)-g) <= detector.force_tolerance_m_s2 && ...
    max(forceStd) < detector.sigma_accel_m_s2 && ...
    max(gyroStd) < detector.sigma_gyro_rad_s && ...
    norm(bg0) < detector.gyro_limit_rad_s, 'RealIMU:InitialMotion', ...
    ['开头 %.2f s 不满足静止初始化条件。请选择静止开始的记录，' ...
     '或调整 InitialStaticSeconds。'], opt.InitialStaticSeconds);
up = f0 / norm(f0);
roll = atan2(up(2), up(3));
pitch = atan2(-up(1), hypot(up(2),up(3)));
q0 = [cos(roll/2)*cos(pitch/2); sin(roll/2)*cos(pitch/2); ...
    cos(roll/2)*sin(pitch/2); -sin(roll/2)*sin(pitch/2)];
% 单一静止姿态只校正沿重力方向的偏差；不能分离全部三轴偏置与倾角。
ba0 = f0 - g * up;
ekf.initial_gyro_bias_rad_s = bg0;
ekf.initial_accel_bias_m_s2 = ba0;
raw = initialState(q0, bg0, ba0, ekf, false);
filt = initialState(q0, bg0, ba0, ekf, true);

rawHistory = history(n);
filterHistory = history(n);
rawHistory = recordState(rawHistory, raw, 1);
filterHistory = recordState(filterHistory, filt, 1);
statistic = inf(n,1);
stationary = false(n,1);
requested = false(n,1);
updated = false(n,1);
rejectedNIS = false(n,1);
nis = nan(n,1);
valid = false(n,1);
impact = false(n,1);
forceCal = D.force_S_m_s2 - ba0';
gyroCal = D.gyro_S_rad_s - bg0';
forceNorm = sqrt(sum(forceCal.^2,2));
gyroNorm = sqrt(sum(gyroCal.^2,2));
% 量程风险按原始轴值判断；不裁剪输入。
accelOverRange = max(abs(D.force_S_m_s2),[],2) >= quality.accelerometer_range_g*g;
gyroOverRange = max(abs(D.gyro_S_rad_s),[],2) >= quality.gyroscope_range_deg_s*pi/180;
impact(:) = forceNorm > quality.impact_force_norm_g*g;
valid(:) = ~(accelOverRange | gyroOverRange | impact);
instantQuiet = valid & gyroNorm <= detector.gyro_limit_rad_s & ...
    abs(forceNorm-g) <= detector.force_tolerance_m_s2;
quietSince = NaN;
windowStart = 1;
lastAttempt = -Inf;
minimumEigenvalue = Inf;
for k = 1:n
    % 因果时间窗口：只看当前及过去数据，掉帧时不跨空缺累计静止时长。
    if instantQuiet(k)
        if isnan(quietSince) || (k > 1 && t(k)-t(k-1) > 5*medianDt)
            quietSince = t(k);
        end
    else
        quietSince = NaN;
    end
    while windowStart < k && t(windowStart+1) <= t(k)-detector.window_s
        windowStart = windowStart + 1;
    end
    idx = windowStart:k;
    if t(k)-t(windowStart) >= detector.window_s && numel(idx) >= 3
        meanForce = mean(forceCal(idx,:),1);
        expected = g * meanForce / max(norm(meanForce),1e-12);
        residual = forceCal(idx,:) - expected;
        statistic(k) = mean(sum(residual.^2,2)/detector.sigma_accel_m_s2^2 + ...
            sum(gyroCal(idx,:).^2,2)/detector.sigma_gyro_rad_s^2);
        stationary(k) = all(instantQuiet(idx)) && ...
            t(k)-quietSince >= detector.minimum_stationary_s && ...
            max(diff(t(idx))) <= 5*medianDt && statistic(k) < detector.glrt_threshold;
    end
    if k == 1, continue; end
    delta = t(k)-t(k-1);
    gyro = D.gyro_S_rad_s(k,:)';
    force = D.force_S_m_s2(k,:)';
    raw = propagate(raw, gyro, force, delta, valid(k), 1, g, ahrs, ekf);
    noiseScale = 1;
    if ~valid(k), noiseScale = quality.impact_noise_multiplier; end
    filt = propagate(filt, gyro, force, delta, valid(k), noiseScale, g, ahrs, ekf);
    requested(k) = strcmpi(opt.ZUPTMode,'glrt') && stationary(k) && ...
        t(k)-lastAttempt+1e-10 >= ekf.zupt_min_interval_s;
    if requested(k)
        lastAttempt = t(k);
        [filt, updated(k), rejectedNIS(k), nis(k)] = zeroVelocityUpdate(filt, ekf);
    end
    rawHistory = recordState(rawHistory, raw, k);
    filterHistory = recordState(filterHistory, filt, k);
    if mod(k,ekf.covariance_check_every) == 0 || k == n
        minimumEigenvalue = min(minimumEigenvalue, min(eig(filt.P)));
        assert(minimumEigenvalue >= -ekf.covariance_negative_tolerance, ...
            'RealIMU:Covariance', 'ESKF 协方差失去半正定性。');
    end
end

result = struct;
result.data_file = char(dataFile);
result.time_s = t;
result.timestamp_ns = D.timestamp_ns;
result.step = D.step;
result.force_S_m_s2 = D.force_S_m_s2;
result.gyro_S_rad_s = D.gyro_S_rad_s;
result.device_quaternion_wxyz = D.device_quaternion_wxyz;
result.raw = rawHistory;
result.eskf = filterHistory;
result.stationary = stationary;
result.glrt_statistic = statistic;
result.zupt_requested = requested;
result.zupt_updated = updated;
result.zupt_rejected_nis = rejectedNIS;
result.zupt_nis = nis;
result.valid_for_aiding = valid;
result.impact = impact;
result.accel_over_range = accelOverRange;
result.gyro_over_range = gyroOverRange;
result.initialization = struct('duration_s',opt.InitialStaticSeconds, ...
    'samples',nnz(cal),'quaternion_wxyz',q0','gyro_bias_rad_s',bg0', ...
    'accel_bias_m_s2',ba0','force_std_m_s2',forceStd,'gyro_std_rad_s',gyroStd);
result.settings = struct('options',opt,'gravity_m_s2',g,'ahrs',ahrs, ...
    'eskf',ekf,'stationary',detector,'quality',quality);
result.covariance_min_eigenvalue = minimumEigenvalue;
result.frame = 'Local W: +Z up, initial yaw = 0; position relative to first sample';
result.has_position_truth = false;
fprintf('\n实测 IMU：%d 帧，%.3f s，中位采样率 %.2f Hz，dt = %.3f~%.3f ms\n', ...
    n,t(end),1/medianDt,1e3*min(dt),1e3*max(dt));
fprintf('初始化：前 %.2f s，%d 帧；起点/初速为零，初始 yaw=0。\n',opt.InitialStaticSeconds,nnz(cal));
fprintf('陀螺仪初始偏置 [rad/s]：[%.6g %.6g %.6g]\n',bg0);
fprintf('沿重力方向初始加速度偏置 [m/s^2]：[%.6g %.6g %.6g]\n',ba0);
fprintf('静止候选 %d 帧；ZUPT 请求/接受/NIS 拒绝 = %d/%d/%d；模式 %s\n', ...
    nnz(stationary),nnz(requested),nnz(updated),nnz(rejectedNIS),char(opt.ZUPTMode));
fprintf('Raw 末帧位置 [m]：[%.4f %.4f %.4f]；末速 %.4f m/s\n',raw.p,norm(raw.v));
fprintf('ESKF 末帧位置 [m]：[%.4f %.4f %.4f]；末速 %.4f m/s\n',filt.p,norm(filt.v));
fprintf('局部坐标系：+Z 向上，初始航向为零；输出为相对起点的估计位移。\n');
if ~any(updated) && strcmpi(opt.ZUPTMode,'glrt')
    warning('RealIMU:NoZUPT','没有成功的零速更新，请检查静止检测曲线和 NIS。');
end
if opt.ShowFigures, plotResult(result); end
end

function ok = positiveScalar(x)
ok = isnumeric(x) && isscalar(x) && isfinite(x) && x > 0;
end

function D = readRecording(path)
assert(isfile(path), 'RealIMU:MissingFile', '找不到数据：%s',path);
fid = fopen(path,'r','n','UTF-8');
assert(fid >= 0, 'RealIMU:OpenFile', '无法打开数据：%s',path);
closer = onCleanup(@() fclose(fid)); %#ok<NASGU>
values = zeros(8192,12);
timestamps = zeros(8192,1,'uint64');
count = 0; lineNumber = 0;
while true
    line = fgetl(fid);
    if ~ischar(line), break; end
    lineNumber = lineNumber + 1;
    line = strtrim(strrep(line,char(65279),''));
    if isempty(line) || line(1) == '#', continue; end
    blocks = strsplit(line,';');
    assert(numel(blocks) == 2, 'RealIMU:Format', ...
        '%s 第 %d 行：需要 IMU 块和 T 块（不接受六 IMU 仿真格式）。',path,lineNumber);
    fields = regexp(strtrim(blocks{1}),'\s+','split');
    numbers = str2double(fields);
    ts = regexp(strtrim(blocks{2}),'^T\s+([0-9]{1,19})$','tokens','once');
    assert(numel(numbers) == 12 && all(isfinite(numbers)) && ...
        numbers(1) >= 1 && numbers(1) == floor(numbers(1)) && numbers(2) == 0 && ...
        ~isempty(ts), 'RealIMU:Format', '%s 第 %d 行：字段格式错误。',path,lineNumber);
    % 先以 uint64 逐位解析，再减首帧，再转 double；保留纳秒时间差精度。
    stamp = uint64(0);
    for digit = double(ts{1})-double('0')
        stamp = stamp*uint64(10) + uint64(digit);
    end
    count = count + 1;
    if count > size(values,1)
        values = [values; zeros(8192,12)]; %#ok<AGROW>
        timestamps = [timestamps; zeros(8192,1,'uint64')]; %#ok<AGROW>
    end
    values(count,:) = numbers;
    timestamps(count) = stamp;
end
assert(count >= 3, 'RealIMU:Samples', '至少需要三帧有效数据。');
values = values(1:count,:);
timestamps = timestamps(1:count);
assert(all(timestamps(2:end) > timestamps(1:end-1)), ...
    'RealIMU:Time', '时间戳必须严格递增，未排序、删除或插值数据。');
assert(all(diff(values(:,1)) > 0), 'RealIMU:Steps', 'step 必须严格递增。');
if any(diff(values(:,1)) ~= 1)
    warning('RealIMU:StepGap','step 存在跳号，仍按真实时间戳处理。');
end
q = values(:,3:6);
qnorm = sqrt(sum(q.^2,2));
assert(all(abs(qnorm-1) < 0.01), 'RealIMU:Quaternion', '设备四元数模长异常。');
D = struct('step',values(:,1),'timestamp_ns',timestamps, ...
    'time_s',double(timestamps-timestamps(1))*1e-9, ...
    'device_quaternion_wxyz',q./qnorm, ...
    'force_S_m_s2',values(:,7:9),'gyro_S_rad_s',values(:,10:12));
end

function state = initialState(q, bg, ba, cfg, withCovariance)
state = struct('p',zeros(3,1),'v',zeros(3,1),'q',q,'bg',bg,'ba',ba, ...
    'acceleration',zeros(3,1),'reliableDuration',0,'P',[]);
if withCovariance
    c = cfg.initial_std;
    sigma = kron([c.position_m; c.velocity_m_s; c.attitude_deg*pi/180; ...
        c.gyro_bias_rad_s; c.accel_bias_m_s2],ones(3,1));
    state.P = diag(sigma.^2);
end
end

function out = history(n)
out = struct('position_W_m',zeros(n,3),'velocity_W_m_s',zeros(n,3), ...
    'quaternion_wxyz',zeros(n,4),'gyro_bias_rad_s',zeros(n,3), ...
    'accel_bias_m_s2',zeros(n,3),'acceleration_W_m_s2',zeros(n,3));
end

function out = recordState(out, s, k)
out.position_W_m(k,:) = s.p';
out.velocity_W_m_s(k,:) = s.v';
out.quaternion_wxyz(k,:) = s.q';
out.gyro_bias_rad_s(k,:) = s.bg';
out.accel_bias_m_s2(k,:) = s.ba';
out.acceleration_W_m_s2(k,:) = s.acceleration';
end

function s = propagate(s, gyro, force, dt, valid, noiseScale, g, ahrs, cfg)
% 同 ins/core.py 的局部右误差 [dp dv dtheta dbg dba] 和重力反馈。
% 实测位置用 p+=v*dt+0.5*a*dt^2；当前样本在该间隔内作常值近似。
omega = gyro-s.bg;
f = force-s.ba;
increment = rotationIncrement(omega*dt);
q = quaternionMultiply(s.q,increment);
R = quaternionMatrix(q);
upS = R(3,:)';
magnitude = norm(f);
direction = f/max(magnitude,1e-12);
angle = acos(max(-1,min(1,dot(direction,upS))));
feedback = valid && abs(magnitude-g) <= ahrs.force_tolerance_m_s2 && ...
    angle <= ahrs.innovation_limit_deg*pi/180;
if feedback
    s.reliableDuration = s.reliableDuration + dt;
else
    s.reliableDuration = 0;
end
jf = eye(3); cf = zeros(3);
if feedback && s.reliableDuration+1e-10 >= ahrs.minimum_reliable_s
    correction = ahrs.gain_s_inv*cross(direction,upS);
    dq = rotationIncrement(correction*dt);
    jr = rightJacobian(correction*dt);
    jf = quaternionMatrix(dq)' + ...
        jr*(ahrs.gain_s_inv*skew(direction)*skew(upS))*dt;
    cf = jr*(ahrs.gain_s_inv*skew(upS)*(eye(3)-direction*direction')/magnitude)*dt;
    q = quaternionMultiply(q,dq);
end
s.q = q/norm(q);
R = quaternionMatrix(s.q);
s.acceleration = R*f + [0;0;-g];
s.p = s.p + s.v*dt + 0.5*s.acceleration*dt^2;
s.v = s.v + s.acceleration*dt;
if isempty(s.P), return; end
jr = rightJacobian(omega*dt);
A = jf*quaternionMatrix(increment)';
B = -jf*jr*dt;
C = cf;
gyroNoise = -jf*jr;
accelNoise = cf/dt;
sensitivity = -R*skew(f);
Phi = eye(15);
Phi(1:3,4:6) = eye(3)*dt;
Phi(7:9,7:9) = A;
Phi(7:9,10:12) = B;
Phi(7:9,13:15) = C;
Phi(4:6,7:9) = sensitivity*A*dt;
Phi(4:6,10:12) = sensitivity*B*dt;
Phi(4:6,13:15) = (sensitivity*C-R)*dt;
Phi(1:3,7:15) = 0.5*dt*Phi(4:6,7:15);
G = zeros(15,12);
G(7:9,1:3) = gyroNoise;
G(7:9,4:6) = accelNoise;
G(4:6,1:3) = sensitivity*gyroNoise*dt;
G(4:6,4:6) = sensitivity*accelNoise*dt-R;
G(1:3,1:6) = 0.5*dt*G(4:6,1:6);
G(10:12,7:9) = eye(3);
G(13:15,10:12) = eye(3);
c = cfg.noise_density;
density = kron([c.gyro_rad_s_sqrt_hz; c.accel_m_s2_sqrt_hz; ...
    c.gyro_bias_random_walk; c.accel_bias_random_walk],ones(3,1));
Qd = diag(density.^2*dt*noiseScale^2);
s.P = Phi*s.P*Phi' + G*Qd*G';
s.P = (s.P+s.P')/2;
assert(all(isfinite(s.P(:))) && all(isfinite([s.p;s.v;s.q;s.bg;s.ba])), ...
    'RealIMU:Nonfinite', 'ESKF 出现非有限状态。');
end

function [s, accepted, rejected, nis] = zeroVelocityUpdate(s, cfg)
H = zeros(3,15); H(:,4:6) = eye(3);
Rm = eye(3)*cfg.zupt_velocity_std_m_s^2;
innovation = -s.v;
S = s.P(4:6,4:6)+Rm;
nis = innovation'*(S\innovation);
accepted = false;
rejected = ~isfinite(nis) || nis > cfg.zupt_nis_limit;
if rejected, return; end
K = (s.P*H')/S;
dx = K*innovation;
M = eye(15)-K*H;
posterior = M*s.P*M'+K*Rm*K'; % Joseph 形式
s.p = s.p+dx(1:3);
s.v = s.v+dx(4:6);
s.q = quaternionMultiply(s.q,rotationIncrement(dx(7:9)));
s.q = s.q/norm(s.q);
s.bg = s.bg+dx(10:12);
s.ba = s.ba+dx(13:15);
reset = eye(15); reset(7:9,7:9) = rightJacobian(dx(7:9));
s.P = reset*posterior*reset';
s.P = (s.P+s.P')/2;
accepted = true;
end

function q = quaternionMultiply(a,b)
q = [a(1)*b(1)-dot(a(2:4),b(2:4)); ...
    a(1)*b(2:4)+b(1)*a(2:4)+cross(a(2:4),b(2:4))];
end

function q = rotationIncrement(v)
angle = norm(v);
if angle < 1e-8
    scale = 0.5-angle^2/48;
else
    scale = sin(angle/2)/angle;
end
q = [cos(angle/2);scale*v];
end

function R = quaternionMatrix(q)
q = q/norm(q);
w=q(1); x=q(2); y=q(3); z=q(4);
R = [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w); ...
    2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w); ...
    2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)];
end

function S = skew(v)
S = [0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end

function J = rightJacobian(v)
angle = norm(v); S = skew(v);
if angle < 1e-6
    J = eye(3)-0.5*S+S*S/6;
else
    J = eye(3)-(1-cos(angle))/angle^2*S+(angle-sin(angle))/angle^3*S*S;
end
end

function plotResult(D)
t = D.time_s;
raw = D.raw.position_W_m;
filtered = D.eskf.position_W_m;
fig = figure('Name','Real IMU - 3D trajectory','Color','w','Position',[80 100 1250 560]);
ax = subplot(1,2,1,'Parent',fig);
drawTrajectory(ax,raw,t,false(size(t)), 'Raw INS (no ZUPT)');
ax = subplot(1,2,2,'Parent',fig);
drawTrajectory(ax,filtered,t,D.zupt_updated, ['ESKF / ' char(D.settings.options.ZUPTMode)]);
% 两个独立坐标范围，防止 Raw 漂移掩盖 ESKF 轨迹；坐标单位始终为米。
rotate3d(fig,'on');

fig = figure('Name','Real IMU - diagnostics','Color','w','Position',[120 70 1150 820]);
ax = subplot(4,1,1,'Parent',fig);
plot(ax,t,filtered,'LineWidth',1.1); grid(ax,'on');
ylabel(ax,'Position (m)'); legend(ax,{'X','Y','Z'},'Location','best');
title(ax,'ESKF relative position; no position ground truth');
ax = subplot(4,1,2,'Parent',fig);
plot(ax,t,sqrt(sum(D.raw.velocity_W_m_s.^2,2)), ...
    t,sqrt(sum(D.eskf.velocity_W_m_s.^2,2)),'LineWidth',1.0);
grid(ax,'on'); ylabel(ax,'Speed (m/s)'); legend(ax,{'Raw','ESKF'},'Location','best');
ax = subplot(4,1,3,'Parent',fig);
yyaxis(ax,'left');
plot(ax,t,sqrt(sum(D.force_S_m_s2.^2,2)));
ylabel(ax,'|f| (m/s^2)');
yyaxis(ax,'right');
plot(ax,t,sqrt(sum(D.gyro_S_rad_s.^2,2)));
ylabel(ax,'|omega| (rad/s)'); grid(ax,'on');
ax = subplot(4,1,4,'Parent',fig);
stairs(ax,t,double(D.stationary),'Color',[.1 .4 .8]); hold(ax,'on');
plot(ax,t(D.zupt_updated),1.1*ones(nnz(D.zupt_updated),1),'g.');
plot(ax,t(D.zupt_rejected_nis),1.2*ones(nnz(D.zupt_rejected_nis),1),'rx');
ylim(ax,[-.1 1.4]); grid(ax,'on'); xlabel(ax,'Time (s)'); ylabel(ax,'Stationary / ZUPT');
legend(ax,{'IMU stationary candidate','ZUPT accepted','NIS rejected'},'Location','best');
linkaxes(findall(fig,'Type','axes'),'x');
end

function drawTrajectory(ax,position,t,updated,name)
plot3(ax,position(:,1),position(:,2),position(:,3),'-','Color',[.65 .65 .65]);
hold(ax,'on');
scatter3(ax,position(:,1),position(:,2),position(:,3),8,t,'filled');
plot3(ax,position(1,1),position(1,2),position(1,3),'go','MarkerFaceColor','g');
plot3(ax,position(end,1),position(end,2),position(end,3),'rs','MarkerFaceColor','r');
plot3(ax,position(updated,1),position(updated,2),position(updated,3),'k.','MarkerSize',5);
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
grid(ax,'on'); axis(ax,'equal'); view(ax,3);
title(ax,name,'Interpreter','none');
cb = colorbar(ax); cb.Label.String = 'Time (s)';
legend(ax,{'Trajectory','Time','Start','End','ZUPT accepted'},'Location','best');
end
