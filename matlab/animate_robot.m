function fig = animate_robot(matFile, varargin)
%ANIMATE_ROBOT Six real-size articulated plates in the original Z-up W frame.
% From the project root: addpath('matlab'); animate_robot
% animate_robot([], 'Stride', 25, 'ReferenceIMU', 0, 'ZUPTMode', 'phase')
% Methods: raw, eskf. Reference choices MUST have precomputed navigation data.
% No orientation/position estimation is performed here. Graphics are created
% once; only transforms, line data and labels are updated during playback.
% Absolute errors use the verified Vi directly: no fitting or initial alignment.
% Requires only base MATLAB. Python exports MAT v5 using scipy.io.savemat.

if nargin < 1 || isempty(matFile)
    projectRoot = fileparts(fileparts(mfilename('fullpath')));
    matFile = fullfile(projectRoot, 'outputs', 'robot', 'robot_odometry.mat');
end
p = inputParser;
p.addParameter('Mode', 'estimated', @(x) ischar(x) || isstring(x));
p.addParameter('Method', 'eskf', @(x) ischar(x) || isstring(x));
p.addParameter('ReferenceIMU', [], @(x) isempty(x) || (isscalar(x) && ismember(x,0:5)));
p.addParameter('ZUPTMode', '', @(x) ischar(x) || isstring(x));
p.addParameter('Stride', 20, @(x) isscalar(x) && isfinite(x) && x >= 1 && x == floor(x));
p.addParameter('PlaybackSpeed', 1, @(x) isscalar(x) && isfinite(x) && x > 0);
p.addParameter('ShowTruth', true, @(x) islogical(x) && isscalar(x));
p.addParameter('ShowLocalAxes', true, @(x) islogical(x) && isscalar(x));
p.addParameter('Visible', 'on', @(x) any(strcmpi(x, {'on','off'})));
p.addParameter('MaxFrames', Inf, @(x) isscalar(x) && x > 0);
p.parse(varargin{:});
opt = p.Results;
assert(isfile(matFile), 'Robot:MissingMAT', 'MAT file missing: %s. Run python main.py first.', matFile);
D = load(matFile);
required = {'schema_version','time_s','controls','modes', ...
    'plate_vertices_S_m','plate_faces_one_based','crack_centers_W_m', ...
    'crack_half_sizes_m','physical_tip_offset_S_m','imu_body_offset_S_m', ...
    'reference_indices_zero_based','default_reference_index','default_policy_one_based'};
for j = 1:numel(required)
    assert(isfield(D, required{j}), 'Robot:Schema', 'Missing MAT field %s.', required{j});
end
assert(D.schema_version == 2, 'Robot:Schema', 'Expected schema 2. Regenerate MAT using the current python main.py.');
t = D.time_s(:)'; n = numel(t);
assert(n >= 2 && all(isfinite(t)) && all(diff(t) > 0), 'Robot:Time', 'Invalid time vector.');
assert(max(abs(diff(t)-0.001)) < 1e-10, 'Robot:Time', 'Expected 1000 Hz unresampled data.');
assert(isequal(size(D.controls), [3 n]) && all(ismember(D.controls(:), [0 1])), 'Robot:Controls', 'Expected 3 x N independent binary controls.');
% Truth is optional display data. Absence is explicitly unavailable, never a
% zero world trajectory. Nonempty malformed arrays still fail dimension checks.
if ~isfield(D,'truth_endpoints_W') || isempty(D.truth_endpoints_W)
    D.truth_endpoints_W = nan(3,7,n);
end
if ~isfield(D,'truth_endpoint_index_zero_based') || isempty(D.truth_endpoint_index_zero_based)
    D.truth_endpoint_index_zero_based = -ones(1,6);
end
assert(isequal(size(D.truth_endpoints_W), [3 7 n]), 'Robot:Schema', 'truth_endpoints_W must be 3 x 7 x N.');
assert(isequal(size(D.truth_endpoint_index_zero_based),[1 6]) && ...
    all(ismember(D.truth_endpoint_index_zero_based, -1:6)), ...
    'Robot:TruthMapping', 'Expected six verified endpoint indices; -1 means unavailable.');
assert(isequal(size(D.plate_vertices_S_m), [3 8 6]), 'Robot:Schema', 'plate_vertices_S_m must be 3 x 8 x 6.');
modeNames = fieldnames(D.modes);
referenceIndices = double(D.reference_indices_zero_based(:)');
assert(all(ismember(referenceIndices,0:5)) && numel(unique(referenceIndices))==numel(referenceIndices), ...
    'Robot:Reference', 'Invalid precomputed reference indices.');
errorCache = struct();
for j = 1:numel(modeNames)
    M = D.modes.(modeNames{j});
    assert(isequal(size(M.R_WS), [3 3 6 n]), 'Robot:Schema', 'R_WS must be 3 x 3 x 6 x N.');
    assert(isequal(size(M.q_WS_wxyz), [4 6 n]) && isequal(size(M.relative_markers_W), [3 7 n]), ...
        'Robot:Schema', 'Invalid shared relative shape dimensions.');
    for reference = referenceIndices
        key = sprintf('S%d',reference);
        assert(isfield(M.references,key), 'Robot:Reference', 'Reference %s not precomputed for %s.',key,modeNames{j});
        Ref = M.references.(key);
        assert(isequal(size(Ref.raw_position_W),[3 n]) && isequal(size(Ref.eskf_position_W),[3 n 3]), ...
            'Robot:Schema', 'Invalid single-reference trajectory dimensions.');
        assert(isequal(size(Ref.eskf_q_WS_wxyz),[4 n 3]) && isequal(size(Ref.covariance_diagonal),[15 n 3]), ...
            'Robot:Schema', 'Invalid ESKF state/covariance dimensions.');
        assert(isequal(size(Ref.zupt_updated),[3 n]) && all(ismember(Ref.zupt_updated(:),[0 1])) && ...
            isequal(size(Ref.zupt_requested),[3 n]) && isequal(size(Ref.glrt_candidate),[1 n]) && ...
            isequal(size(Ref.phase_glrt_candidate),[1 n]) && ...
            all(isfinite(Ref.raw_position_W(:))) && all(isfinite(Ref.eskf_position_W(:))), ...
            'Robot:Schema', 'Invalid update flags or navigation values.');
        % Cache is display-only. Missing/unconfirmed truth stays NaN, never zero.
        truthIndex = double(D.truth_endpoint_index_zero_based(reference+1));
        truthPosition = nan(3,n);
        if truthIndex>=0
            truthPosition = reshape(D.truth_endpoints_W(:,truthIndex+1,:),3,n);
        end
        errorCache.(modeNames{j}).(key) = positionErrorCache(Ref,truthPosition);
    end
end
modeIndex = find(strcmp(modeNames, char(opt.Mode)), 1);
assert(~isempty(modeIndex), 'Robot:Mode', 'Mode %s is not exported. Available: %s.', char(opt.Mode), strjoin(modeNames, ', '));
methodKeys = {'raw','eskf'};
methodLabels = {'Raw INS placement', 'Online ESKF placement'};
methodIndex = find(strcmpi(methodKeys, char(opt.Method)), 1);
assert(~isempty(methodIndex), 'Robot:Method', 'Method must be raw or eskf.');
if isempty(opt.ReferenceIMU), reference = double(D.default_reference_index); else, reference = opt.ReferenceIMU; end
referenceMenuIndex = find(referenceIndices==reference,1);
assert(~isempty(referenceMenuIndex), 'Robot:Reference', ...
    'S%d is not precomputed. Run python main.py --reference-imu %d, or --precompute-references with that index.',reference,reference);
referenceLabels = arrayfun(@(x) sprintf('S%d',x), referenceIndices, 'UniformOutput',false);
policyKeys = {'none','glrt','phase'};
policyLabels = {'No ZUPT', 'GLRT only', 'Phase-assisted GLRT'};
if isempty(opt.ZUPTMode)
    policyIndex = double(D.default_policy_one_based);
else
    policyIndex = find(strcmpi(policyKeys,char(opt.ZUPTMode)),1);
end
assert(~isempty(policyIndex) && ismember(policyIndex,1:3), 'Robot:Policy', 'ZUPTMode must be none, glrt or phase.');

fig = figure('Name', 'Multi-IMU origami robot: W frame, SI units', ...
    'Color', 'w', 'Position', [60 60 1520 900], 'Visible', opt.Visible);
ax = axes('Parent', fig, 'Position', [.04 .21 .52 .66]);
hold(ax, 'on'); grid(ax, 'on'); view(ax, 40, 25); axis(ax, 'equal');
xlabel(ax, 'X_W (m)'); ylabel(ax, 'Y_W (m)'); zlabel(ax, 'Z_W (m)');
colors = lines(6);
faces = double(D.plate_faces_one_based);

% The crack boxes use XML coordinates and sizes, not a decorative rescaling.
for j = 1:2
    vertices = boxVertices(D.crack_centers_W_m(:,j), D.crack_half_sizes_m(:,j));
    patch('Parent', ax, 'Faces', faces, 'Vertices', vertices, ...
        'FaceColor', [.45 .45 .48], 'FaceAlpha', .14, 'EdgeAlpha', .25);
    text(ax, D.crack_centers_W_m(1,j), D.crack_centers_W_m(2,j), ...
        D.crack_centers_W_m(3,j)+D.crack_half_sizes_m(3,j)+.012, ...
        sprintf('crack%d',j), 'Color', [.3 .3 .3]);
end
worldColors = eye(3);
for component = 1:3
    endpoint = .065 * worldColors(component,:);
    line(ax, [0 endpoint(1)], [0 endpoint(2)], [0 endpoint(3)], ...
        'Color', worldColors(component,:), 'LineWidth', 1.8);
    labels = {'X_W','Y_W','Z_W'};
    text(ax, endpoint(1), endpoint(2), endpoint(3), labels{component});
end
text(ax, 0,0,0, ' W(0,0,0)', 'FontSize', 8);

plateTransform = gobjects(6,1); localAxes = gobjects(6,3);
imuLabels = gobjects(6,1); connectors = gobjects(6,1);
for node = 1:6
    plateTransform(node) = hgtransform('Parent', ax);
    patch('Parent', plateTransform(node), 'Faces', faces, ...
        'Vertices', D.plate_vertices_S_m(:,:,node)', 'FaceColor', colors(node,:), ...
        'FaceAlpha', .87, 'EdgeColor', colors(node,:)*.5);
    % Geometry is X=50mm, Y=2mm thickness, Z=50mm width, exactly as MJCF.
    for component = 1:3
        endpoint = .014 * worldColors(component,:);
        origin = D.imu_body_offset_S_m(:,node)';
        localAxes(node,component) = line('Parent', plateTransform(node), ...
            'XData', [origin(1) origin(1)+endpoint(1)], ...
            'YData', [origin(2) origin(2)+endpoint(2)], ...
            'ZData', [origin(3) origin(3)+endpoint(3)], ...
            'Color', worldColors(component,:), 'LineWidth', 1.2);
    end
    imuLabels(node) = text(ax, 0,0,0, sprintf('S%d', node-1), 'FontSize', 8, 'Color', colors(node,:)*.65);
    connectors(node) = line(ax, nan,nan,nan, 'Color', [.15 .15 .15], 'LineWidth', 1.1);
end
markerLine = line(ax, nan,nan,nan, 'Color', [.1 .1 .1], 'Marker', '.', 'LineStyle', 'none');
rawTrail = line(ax, nan,nan,nan, 'Color', [.1 .4 .9], 'LineWidth', 1.2);
eskfTrail = line(ax, nan,nan,nan, 'Color', [.9 .3 .1], 'LineWidth', 1.5, 'LineStyle', '--');
truthOutline = line(ax, nan,nan,nan, 'Color', [0 .2 .6], 'LineStyle', '--', 'LineWidth', 1);
truthTrail = line(ax, nan,nan,nan, 'Color', [.1 .3 .6], 'LineStyle', ':', 'LineWidth', 1);
referenceMarker = line(ax, nan,nan,nan, 'Color', [.3 .3 0], 'Marker', 'o', ...
    'MarkerFaceColor', [1 .8 0], 'MarkerSize', 8, 'LineStyle', 'none');
legend(ax,[rawTrail eskfTrail truthTrail referenceMarker], ...
    {'Reference Raw INS','Reference ESKF + selected ZUPT','Reference truth (evaluation)','Current reference position'}, ...
    'Location','northeast','FontSize',8);
status = title(ax, '', 'Interpreter', 'none', 'FontSize',8);

% Two error panels, all graphics allocated once. Markers use successful
% updates only; GLRT candidates and requested updates never create markers.
normAxes = axes('Parent',fig,'Position',[.65 .67 .32 .22]);
hold(normAxes,'on'); grid(normAxes,'on');
xlabel(normAxes,'Time (s)'); ylabel(normAxes,'3D position error (m)');
normTitle = title(normAxes,'Absolute position error');
rawNormLine = line(normAxes,nan,nan,'Color',[.1 .4 .9],'LineWidth',1.2);
eskfNormLine = line(normAxes,nan,nan,'Color',[.9 .3 .1],'LineWidth',1.5,'LineStyle','--');
normUpdateLine = line(normAxes,nan,nan,'Color',[0 .55 .15], ...
    'LineStyle',':','LineWidth',.7);
legend(normAxes,[rawNormLine eskfNormLine normUpdateLine], ...
    {'Raw INS','ESKF-ZUPT','Successful ZUPT update'},'Location','best','FontSize',8);
componentAxes = axes('Parent',fig,'Position',[.65 .39 .32 .19]);
hold(componentAxes,'on'); grid(componentAxes,'on');
xlabel(componentAxes,'Time (s)'); ylabel(componentAxes,'Signed position error (m)');
componentTitle = title(componentAxes,'World X/Y/Z error');
rawComponentLines = gobjects(1,3); eskfComponentLines = gobjects(1,3);
componentColors = [.8 .15 .15; .1 .55 .2; .15 .3 .85];
for component=1:3
    rawComponentLines(component) = line(componentAxes,nan,nan, ...
        'Color',componentColors(component,:),'LineStyle',':','LineWidth',1);
    eskfComponentLines(component) = line(componentAxes,nan,nan, ...
        'Color',componentColors(component,:),'LineStyle','-','LineWidth',1.2);
end
componentUpdateLine = line(componentAxes,nan,nan,'Color',[0 .55 .15], ...
    'LineStyle',':','LineWidth',.7,'HandleVisibility','off');
legend(componentAxes,[rawComponentLines eskfComponentLines], ...
    {'Raw X','Raw Y','Raw Z','ESKF X','ESKF Y','ESKF Z'}, ...
    'Location','best','NumColumns',3,'FontSize',7);
xlim(normAxes,[t(1) t(n)]); xlim(componentAxes,[t(1) t(n)]);
errorStats = uicontrol(fig,'Style','text','Units','normalized', ...
    'Position',[.61 .145 .38 .165],'BackgroundColor','w', ...
    'HorizontalAlignment','left','FontSize',9,'String','');
lastErrorSelection = '';

% Fixed world bounds: no per-frame limits, axis swaps or forced -X sign.
bounds = reshape(D.truth_endpoints_W, 3, []);
for j = 1:numel(modeNames)
    M = D.modes.(modeNames{j});
    for reference = referenceIndices
        Ref = M.references.(sprintf('S%d',reference));
        relative = M.relative_markers_W - M.relative_markers_W(:,reference+1,:);
        for policy = 1:3
            points = relative + reshape(Ref.eskf_position_W(:,:,policy),3,1,n);
            bounds = [bounds, min(reshape(points,3,[]),[],2), max(reshape(points,3,[]),[],2)]; %#ok<AGROW>
        end
        points = relative + reshape(Ref.raw_position_W,3,1,n);
        bounds = [bounds, min(reshape(points,3,[]),[],2), max(reshape(points,3,[]),[],2)]; %#ok<AGROW>
    end
end
bounds = [bounds, D.crack_centers_W_m-D.crack_half_sizes_m, D.crack_centers_W_m+D.crack_half_sizes_m];
bounds = bounds(:,all(isfinite(bounds),1)); % unavailable truth must not poison display bounds
lo = min(bounds,[],2)-.04; hi = max(bounds,[],2)+.04;
xlim(ax, [lo(1) hi(1)]); ylim(ax, [lo(2) hi(2)]); zlim(ax, [lo(3) hi(3)]);
axis(ax, 'manual');

modeMenu = uicontrol(fig, 'Style','popupmenu', 'Units','normalized', ...
    'Position',[.03 .09 .15 .035], 'String',modeNames, 'Value',modeIndex, 'Callback',@refresh);
methodMenu = uicontrol(fig, 'Style','popupmenu', 'Units','normalized', ...
    'Position',[.19 .09 .16 .035], 'String',methodLabels, 'Value',methodIndex, 'Callback',@refresh);
referenceMenu = uicontrol(fig, 'Style','popupmenu', 'Units','normalized', ...
    'Position',[.36 .09 .10 .035], 'String',referenceLabels, 'Value',referenceMenuIndex, 'Callback',@refresh);
policyMenu = uicontrol(fig, 'Style','popupmenu', 'Units','normalized', ...
    'Position',[.47 .09 .14 .035], 'String',policyLabels, 'Value',policyIndex, 'Callback',@refresh);
truthBox = uicontrol(fig, 'Style','checkbox', 'Units','normalized', ...
    'Position',[.62 .09 .13 .035], 'String','Truth overlay', 'Value',opt.ShowTruth, 'Callback',@refresh);
axesBox = uicontrol(fig, 'Style','checkbox', 'Units','normalized', ...
    'Position',[.76 .09 .1 .035], 'String','IMU axes', 'Value',opt.ShowLocalAxes, 'Callback',@refresh);
pauseButton = uicontrol(fig, 'Style','togglebutton', 'Units','normalized', ...
    'Position',[.89 .09 .08 .035], 'String','Pause', 'Value',0);
frameSlider = uicontrol(fig, 'Style','slider', 'Units','normalized', ...
    'Position',[.07 .04 .89 .025], 'Min',1, 'Max',n, 'Value',1, ...
    'SliderStep',[min(1,opt.Stride/(n-1)) .1], 'Callback',@scrub);
uicontrol(fig, 'Style','text', 'Units','normalized', 'Position',[.04 .145 .54 .035], ...
    'String','W: +Z up; original -X motion | Six-AHRS relative shape + selected reference INS translation | No geometry position constraints', ...
    'BackgroundColor','w', 'HorizontalAlignment','left');

currentFrame = 1; rendered = 0;
objectCount = numel(findall(fig));
setappdata(fig, 'InitialObjectCount', objectCount);
setappdata(fig, 'MATFile', char(matFile));
while isgraphics(fig) && rendered < opt.MaxFrames
    if get(pauseButton,'Value')
        drawnow; pause(.02); continue;
    end
    tick = tic;
    renderedFrame = currentFrame;
    renderFrame(currentFrame);
    rendered = rendered + 1;
    if currentFrame >= n, break; end
    nextFrame = min(n, currentFrame + opt.Stride);
    duration = (t(nextFrame)-t(currentFrame))/opt.PlaybackSpeed;
    drawnow limitrate;
    pause(max(0, duration-toc(tick)));
    if ~isgraphics(fig), break; end
    % A scrub callback during drawnow/pause owns the new frame. Do not jump
    % back to the previously scheduled frame; keep pause at the visible frame.
    if currentFrame==renderedFrame && ~get(pauseButton,'Value') && rendered<opt.MaxFrames
        currentFrame = nextFrame;
    end
end
if isgraphics(fig)
    drawnow;
    setappdata(fig, 'FinalObjectCount', numel(findall(fig)));
    setappdata(fig, 'FramesRendered', rendered);
    setappdata(fig, 'CurrentFrame', currentFrame);
end

    function renderFrame(frame)
        name = modeNames{get(modeMenu,'Value')}; M = D.modes.(name);
        method = get(methodMenu,'Value');
        reference = referenceIndices(get(referenceMenu,'Value'));
        Ref = M.references.(sprintf('S%d',reference));
        policy = get(policyMenu,'Value');
        if method==1
            referencePosition = Ref.raw_position_W(:,frame);
        else
            referencePosition = Ref.eskf_position_W(:,frame,policy);
        end
        % Rendering only: never feed relative geometry back to the reference INS.
        root = referencePosition - M.relative_markers_W(:,reference+1,frame) - ...
            M.R_WS(:,:,reference+1,frame)*D.imu_body_offset_S_m(:,reference+1);
        points = M.relative_markers_W(:,:,frame) + root;
        for i = 1:6
            R = M.R_WS(:,:,i,frame);
            T = eye(4); T(1:3,1:3) = R; T(1:3,4) = points(:,i);
            set(plateTransform(i), 'Matrix', T);
            physicalTip = points(:,i) + R*D.physical_tip_offset_S_m;
            updateLine(connectors(i), [physicalTip, points(:,i+1)]);
            imu = points(:,i) + R*D.imu_body_offset_S_m(:,i);
            set(imuLabels(i), 'Position', (imu+[0;0;.006])');
            set(localAxes(i,:), 'Visible', onOff(get(axesBox,'Value')));
        end
        updateLine(markerLine, points);
        indices = 1:frame; % all trajectory/error samples through the same frame
        updateLine(rawTrail, Ref.raw_position_W(:,indices));
        updateLine(eskfTrail, Ref.eskf_position_W(:,indices,policy));
        updateLine(truthOutline, D.truth_endpoints_W(:,:,frame));
        truthIndex = double(D.truth_endpoint_index_zero_based(reference+1));
        if truthIndex>=0
            updateLine(truthTrail, reshape(D.truth_endpoints_W(:,truthIndex+1,indices),3,[]));
        end
        set(truthOutline, 'Visible', onOff(get(truthBox,'Value')));
        set(truthTrail, 'Visible', onOff(get(truthBox,'Value') && truthIndex>=0));
        updateLine(referenceMarker, referencePosition);
        updated = logical(Ref.zupt_updated(policy,frame));
        if updated
            set(referenceMarker,'MarkerFaceColor',[.1 .9 .2]);
        else
            set(referenceMarker,'MarkerFaceColor',[1 .8 0]);
        end
        set(status, 'String', sprintf(['t=%.3f s | %s / S%d / %s\n' ...
            '%s | u=[%d %d %d] | p_ref=[%.3f %.3f %.3f] m\n' ...
            'GLRT=%d, phase-GLRT=%d, requested=%d, actual ZUPT=%d, aiding valid=%d'], ...
            t(frame),name,reference,methodKeys{method},policyLabels{policy}, ...
            D.controls(1,frame),D.controls(2,frame),D.controls(3,frame), ...
            referencePosition(1),referencePosition(2),referencePosition(3), ...
            Ref.glrt_candidate(frame),Ref.phase_glrt_candidate(frame), ...
            Ref.zupt_requested(policy,frame),updated,Ref.quality.valid_for_aiding(frame)));
        renderErrors(name,reference,policy,method,frame);
        set(frameSlider, 'Value', frame);
        setappdata(fig, 'CurrentFrame', frame);
        setappdata(fig, 'RootPosition', root);
        setappdata(fig, 'ReferenceIMUIndex', reference);
        setappdata(fig, 'ReferencePosition', referencePosition);
    end

    function renderErrors(name,reference,policy,method,frame)
        key = sprintf('S%d',reference);
        E = errorCache.(name).(key);
        Ref = D.modes.(name).references.(key);
        selection = sprintf('%s/%s/%d/%d',name,key,policy,method);
        if ~strcmp(selection,lastErrorSelection)
            ylim(normAxes,errorLimits([E.raw3D E.eskf3D(policy,:)],true));
            values = [E.rawXYZ,reshape(E.eskfXYZ(:,:,policy),3,n)];
            ylim(componentAxes,errorLimits(values,false));
            if E.hasTruth
                suffix = '';
            else
                suffix = ' | N/A: no verified finite truth';
            end
            set(normTitle,'String',sprintf('%s / %s / %s%s',name,key,policyKeys{policy},suffix), ...
                'Interpreter','none');
            set(componentTitle,'String',['World X/Y/Z error (no alignment)' suffix],'Interpreter','none');
            lastErrorSelection = selection;
        end
        prefix = 1:frame;
        set(rawNormLine,'XData',t(prefix),'YData',E.raw3D(prefix),'LineWidth',1.2+double(method==1));
        set(eskfNormLine,'XData',t(prefix),'YData',E.eskf3D(policy,prefix),'LineWidth',1.2+double(method==2));
        for component=1:3
            set(rawComponentLines(component),'XData',t(prefix),'YData',E.rawXYZ(component,prefix));
            set(eskfComponentLines(component),'XData',t(prefix),'YData',E.eskfXYZ(component,prefix,policy));
        end
        % Vertical event segments are one line object per axes. An empty set
        % of successful updates produces empty data, including when count=0.
        updateTimes = t(prefix(logical(Ref.zupt_updated(policy,prefix))));
        updateEventLine(normUpdateLine,updateTimes,ylim(normAxes));
        updateEventLine(componentUpdateLine,updateTimes,ylim(componentAxes));
        statLines = {
            sprintf('Selected placement: %s | %s / %s',methodKeys{method},key,policyKeys{policy})
            sprintf('Current 3D Position Error (m): Raw %s | ESKF %s',errorValue(E.raw3D(frame)),errorValue(E.eskf3D(policy,frame)))
            sprintf('Cumulative RMSE to %.3f s (m): Raw %s | ESKF %s',t(frame),errorValue(E.rawRMSE(frame)),errorValue(E.eskfRMSE(policy,frame)))
            sprintf('Dataset Final Position Error at %.3f s (m): Raw %s | ESKF %s',t(n),errorValue(E.raw3D(n)),errorValue(E.eskf3D(policy,n)))
            sprintf('ZUPT Updates: %d (through current frame) | Dataset total: %d',E.updateCount(policy,frame),E.updateCount(policy,n))
            sprintf('Finite truth samples: %d / %d | Full-dataset RMSE: Python summary.json',E.validCount(frame),frame)
            'Absolute error can include initial-state error; no trajectory alignment.'};
        set(errorStats,'String',statLines);
        setappdata(fig,'ErrorDisplayFrame',frame);
        setappdata(fig,'CurrentRaw3DError',E.raw3D(frame));
        setappdata(fig,'CurrentESKF3DError',E.eskf3D(policy,frame));
        setappdata(fig,'CumulativeRawRMSE',E.rawRMSE(frame));
        setappdata(fig,'CumulativeESKFRMSE',E.eskfRMSE(policy,frame));
        setappdata(fig,'ZUPTUpdateCount',E.updateCount(policy,frame));
    end

    function refresh(~,~)
        if isgraphics(fig), renderFrame(currentFrame); drawnow limitrate; end
    end

    function scrub(source,~)
        currentFrame = max(1,min(n,round(get(source,'Value'))));
        renderFrame(currentFrame); drawnow limitrate;
    end
end

function updateLine(handle, points)
set(handle, 'XData',points(1,:), 'YData',points(2,:), 'ZData',points(3,:));
end

function value = onOff(condition)
if condition, value = 'on'; else, value = 'off'; end
end

function vertices = boxVertices(center, halfSize)
signs = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1];
vertices = signs .* halfSize(:)' + center(:)';
end

function E = positionErrorCache(Ref,truth)
% Only actual world-position subtraction. Never feed these arrays to INS or
% the hgtransform placement. NaN gaps remain gaps in plotted error values.
n = size(truth,2);
valid = all(isfinite(truth),1);
E.hasTruth = any(valid);
E.validCount = cumsum(valid);
E.rawXYZ = Ref.raw_position_W - truth;
E.rawXYZ(:,~valid) = NaN;
E.raw3D = sqrt(sum(E.rawXYZ.^2,1));
E.rawRMSE = prefixRMSE(E.raw3D,valid);
E.eskfXYZ = nan(3,n,3);
E.eskf3D = nan(3,n); E.eskfRMSE = nan(3,n);
for policy=1:3
    error = Ref.eskf_position_W(:,:,policy) - truth;
    error(:,~valid) = NaN;
    E.eskfXYZ(:,:,policy) = error;
    E.eskf3D(policy,:) = sqrt(sum(error.^2,1));
    E.eskfRMSE(policy,:) = prefixRMSE(E.eskf3D(policy,:),valid);
end
E.updateCount = cumsum(double(Ref.zupt_updated),2);
end

function value = prefixRMSE(error3D,valid)
% With complete truth: sqrt(mean(sum((p_est-p_gt).^2,1))) on each prefix.
% Missing truth is excluded only from the running statistic, not replaced by
% a fake zero error in any curve; the UI reports the finite-sample count.
squared = error3D.^2;
squared(~valid) = 0;
count = cumsum(valid); total = cumsum(squared);
value = nan(size(error3D)); available = count>0;
value(available) = sqrt(total(available)./count(available));
end

function value = errorValue(error)
if isfinite(error), value=sprintf('%.5g',error); else, value='N/A'; end
end

function limits = errorLimits(values,nonnegative)
% Display limits only: do not rescale, normalize or offset any error samples.
values = values(isfinite(values));
if isempty(values)
    if nonnegative, limits=[0 1]; else, limits=[-1 1]; end
elseif nonnegative
    limits = [0 max(max(values)*1.08,1e-12)];
else
    low=min(values); high=max(values);
    padding=max((high-low)*.08,max(abs([low high]))*.01);
    padding=max(padding,1e-12);
    limits=[low-padding high+padding];
end
end

function updateEventLine(handle,times,limits)
if isempty(times)
    set(handle,'XData',[],'YData',[]);
else
    x = [times;times;nan(size(times))];
    y = [repmat(limits(1),size(times));repmat(limits(2),size(times));nan(size(times))];
    set(handle,'XData',x(:)','YData',y(:)');
end
end
