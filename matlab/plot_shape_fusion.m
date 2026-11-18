function figures = plot_shape_fusion(folder, noiseMode, algorithm)
%PLOT_SHAPE_FUSION Plot native CSV results; no estimation or truth feedback here.
% plot_shape_fusion('outputs/shape','full','A2')
if nargin<1 || isempty(folder)
    error('请传入新运行实验的输出目录。');
end
if nargin < 2, noiseMode = 'full'; end
if nargin < 3, algorithm = 'A2'; end
J=readtable(fullfile(folder,'joint_angles.csv'));
S=readtable(fullfile(folder,'shape_errors.csv'));
N=readtable(fullfile(folder,'navigation_errors.csv'));
select=@(T) strcmp(string(T.noise),noiseMode) & strcmp(string(T.algorithm),algorithm);
jm=select(J); sm=select(S); nm=select(N);
assert(any(jm),'Requested noise/algorithm has no completed samples; inspect summary.json.');
figures=gobjects(3,1);
figures(1)=figure('Name','Five-joint estimates');
for j=0:4
    subplot(3,2,j+1); mask=jm & J.joint==j;
    plot(J.time_s(mask),J.truth_rad(mask)*180/pi,'k-', ...
         J.time_s(mask),J.estimate_rad(mask)*180/pi,'b--');
    grid on; xlabel('Time (s)'); ylabel('Angle (deg)'); title(sprintf('Joint %d',j));
    legend('Truth (evaluation)','Estimated','Location','best');
end
figures(2)=figure('Name','Relative shape errors'); hold on;
for panel=1:5
    mask=sm & S.panel==panel;
    plot(S.time_s(mask),S.error_relative_world_m(mask)*1000, ...
        'DisplayName',sprintf('S%d relative to S0',panel));
end
xlabel('Time (s)'); ylabel('Relative position error (mm)'); grid on; legend('show');
figures(3)=figure('Name','Navigation comparisons'); hold on;
for name={'D0','D1','D2','ORACLE_D1','ORACLE_D2'}
    mask=nm & strcmp(string(N.method),name{1});
    if any(mask), plot(N.time_s(mask),N.error_m(mask),'DisplayName',name{1}); end
end
xlabel('Time (s)'); ylabel('World-frame position error (m)'); grid on; legend('show');
for k=1:3
    print(figures(k),fullfile(folder,sprintf('%s_%s_plot_%d.png',noiseMode,algorithm,k)),'-dpng','-r180');
end
end
