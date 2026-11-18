function fig = plot_multi_imu_eskf(folder)
%PLOT_MULTI_IMU_ESKF One PNG: 3-D overview and enlarged multi-IMU projections.
% plot_multi_imu_eskf('outputs/joint');
if nargin<1 || isempty(folder)
    error('请传入新运行实验的输出目录。');
end
S=jsondecode(fileread(fullfile(folder,'summary.json')));
T=readtable(fullfile(folder,'navigation_errors.csv'));
names={'B0_RAW','B3_D1_A2','D3a','D3b'};
labels={'单 IMU 积分','多 IMU 均值融合','铰链位置约束','铰链位置与轴约束'};
colors=[.47 .47 .47;0 .45 .70;.90 .62 0;0 .62 .45];
styles={'--','-','-.','-'};
truth=T(strcmp(string(T.method),names{1}),:);
fig=figure('Color','w','Position',[60 60 1350 850]);
overview=subplot(2,2,[1 3]);hold on;grid on;
plot3(truth.truth_x_m,truth.truth_y_m,truth.truth_z_m,'k-','LineWidth',2,'DisplayName','真实轨迹');
for k=1:numel(names)
    rows=T(strcmp(string(T.method),names{k}),:);
    name=sprintf('%s (ATE %.3f m)',labels{k},S.metrics.(names{k}).ate_m);
    plot3(rows.x_m,rows.y_m,rows.z_m,'Color',colors(k,:),'LineStyle',styles{k},'LineWidth',1.3,'DisplayName',name);
    scatter3(rows.x_m(end),rows.y_m(end),rows.z_m(end),25,colors(k,:),'filled','HandleVisibility','off');
end
xlabel('X (m)');ylabel('Y (m)');zlabel('Z (m)');axis equal;view(-58,23);
title('三维轨迹总览');legend(overview,'Location','southoutside');
for projection=1:2
    subplot(2,2,2*projection);hold on;grid on;
    if projection==1, truthY=truth.truth_y_m; else, truthY=truth.truth_z_m; end
    plot(truth.truth_x_m,truthY,'k-','LineWidth',2);
    for k=2:numel(names)
        rows=T(strcmp(string(T.method),names{k}),:);
        if projection==1, y=rows.y_m; else, y=rows.z_m; end
        plot(rows.x_m,y,'Color',colors(k,:),'LineStyle',styles{k},'LineWidth',1.4);
        scatter(rows.x_m(end),y(end),25,colors(k,:),'filled');
    end
    xlabel('X (m)');axis equal;
    if projection==1, ylabel('Y (m)');title('多 IMU 轨迹局部放大 (XY)');
    else, ylabel('Z (m)');title('多 IMU 轨迹局部放大 (XZ)');end
end
print(fig,fullfile(folder,'trajectory.png'),'-dpng','-r200');
end
