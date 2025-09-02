kin = hardcoded_IK_setups.IRB_6640.get_kin();
kin.P(:,end) = 0; % Place task frame at spherical wrist

Z = 1.5; % Height of plane for task path
r_23 = vecnorm(kin.P(:,3));
r_34 = vecnorm(kin.P(:,4));

% Calculate radius on plane of workspace boundary

h = Z - kin.P(3,2);
R = r_23+r_34;
w = sqrt(R^2-h^2);
D1 = kin.P(1,2) + w;
D2 = -kin.P(1,2) + w;

ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];


%% Plot out side view


diagrams.setup([2,3]); hold on
view(2);
diagrams.circle(kin.P([1 3 2],2), ez, r_23+r_34);
diagrams.circle(kin.P([1 3 2],2), ez, r_23-r_34);
diagrams.circle([-1; 1; 1].*kin.P([1 3 2],2), ez, r_23+r_34);
diagrams.circle([-1; 1; 1].*kin.P([1 3 2],2), ez, r_23-r_34);

diagrams.dot(Z*ey + D1*ex);
diagrams.dot(Z*ey + D2*ex);

diagrams.line(-3*ex+Z*ey, 3*ex+Z*ey);
diagrams.line(-2*ey,3.5*ey, lineStyle="-.");

diagrams.arrow(-ey, ey);
diagrams.arrow(-ex, ex);

diagrams.redraw(); hold off

%% Generate path in XY plane
r_outer = D1;

N = 1e2;
p_path = NaN([3 4*N]);
e_sing_path = NaN([3 4*N]);

% Part 1: outer arc

theta_path = linspace(deg2rad(30), deg2rad(150), N);

for i = 1:N
    p_path(:,i) = rot(ez, theta_path(i))*r_outer*ex;
    e_sing_path(:,i) = rot(ez, theta_path(i))*ex;
end

% Part 2: Approach center
lambda = linspace(0,1,N);
p_A = rot(ez, deg2rad(150))*r_outer*ex;
p_path(:, N+1:2*N) =  (1-lambda).*p_A;
for i = N+1:2*N
    e_sing_path(:,i) = rot(ez, deg2rad(150))*ey;
end

% Part 3: Orbit center
% Hold position for N samples
% Rotate e_sing
theta_path = linspace(deg2rad(150), deg2rad(30+180), N);
for i = 1:N
    p_path(:,2*N+i) = zv;
    e_sing_path(:,2*N+i) = rot(ez, theta_path(i))*ey;
end

% Part 4: Approach outer arc
p_B = rot(ez, deg2rad(30))*r_outer*ex;
p_path(:, 3*N+1:4*N) = lambda.*p_B;
for i = 3*N+1:4*N
    e_sing_path(:,i) = rot(ez, deg2rad(30+180))*ey;
end

p_path(3,:) = Z;

%% Plot path on XY plane

diagrams.setup([2,3]); hold on
view(2);
camproj('orthographic')

% diagrams.circle(zv, ez, D1);
draw_arc(r_outer*rot(ez, deg2rad(30))*ex, ez, ez, deg2rad(120), color=diagrams.colors.red);
diagrams.line(zv, r_outer*rot(ez, deg2rad(30))*ex, color=diagrams.colors.red)
diagrams.line(zv, r_outer*rot(ez, deg2rad(150))*ex, color=diagrams.colors.red)
draw_arc(r_outer*rot(ez, deg2rad(150))*ex, ez, ez, deg2rad(240), lineStyle=":");
diagrams.circle(zv, ez, D2, lineStyle=":");

% diagrams.utils.plot3_mat(p_path, color = diagrams.colors.red);
% for i = 1:25:4*N
%     diagrams.arrow(p_path(:,i), p_path(:,i)+e_sing_path(:,i)*0.2, color=diagrams.colors.red);
% end

% diagrams.arrow(-ey, ey);
% diagrams.arrow(-ex, ex);
diagrams.redraw(); hold off

%%  IK on the whole path

Q_path = NaN(6,8,length(p_path));
is_LS_path = NaN(8,length(p_path));
for i = 1:length(p_path)
    [Q, is_LS] = IK_spherical_2_parallel_sing(eye(3), p_path(:,i), kin, e_sing_path(:,i));
    Q_path(:,1:width(Q), i) = Q;
    is_LS_path(1:width(is_LS), i) = is_LS;
end

%%

plot(squeeze(Q_path(5,:,:))', '.');
%% 4 different continuous q_path with q5 > 0

q_path_A = [squeeze(Q_path(:, 5, 1:199)) squeeze(Q_path(:, 1, 200:400))];
q_path_B = [squeeze(Q_path(:, 3, 1:199)) squeeze(Q_path(:, 7, 200:400))];
q_path_C = [squeeze(Q_path(:, 7, 1:199)) squeeze(Q_path(:, 3, 200:400))];
q_path_D = [squeeze(Q_path(:, 1, 1:199)) squeeze(Q_path(:, 5, 200:400))];

plot(squeeze(Q_path(5,:,:))', 'k.'); hold on
plot(q_path_D(5, :), 'r', LineWidth=5);

hold off
%%
plot(is_LS_path', 'x')
%%
plot(squeeze(Q_path(5,:,:))', 'k');
%% Plot of all q5 solutions
h_fig = figure(10);
% tiledlayout(1,1,'TileSpacing','none','Padding','compact');
% nexttile
figure_size = 2*[3.5 3];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs
set(h_fig, 'renderer', 'painters')

q5_disp = squeeze(Q_path(5,:,:));


q5_LS = q5_disp;
q5_LS(~is_LS_path) = NaN;

plot(q5_disp(:,1:2:end)', 'k.', MarkerSize =2); hold on
plot(q5_LS(:,1:2:end)',   '.', MarkerSize =8, color=diagrams.colors.black);
hold off

xlabel('$k$', Interpreter="latex");
ylabel('$q_5$', Interpreter="latex");
legend("Non-LS", "", "", "", "", "", "", "","LS", Interpreter="latex", Location='west');

ylim([-pi, pi]);
yticks([-pi, -pi/2, 0, pi/2, pi]);
yticklabels({'$-\pi$', '$-\frac{\pi}{2}$', '$0$', '$\frac{\pi}{2}$', '$\pi$'})


xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

fontsize(gca, 2*10,"points")
%%
diagrams.save(gcf, 'IRB_q_plot')
%% Single pose
rob_opts = {'show_base_label', false,...
        'show_task_label', false,...
        'show_base_frame', false,...
        'show_task_frame', false,...
        'show_arrow_labels', false,...
        'show_joint_labels', false,...
        'show_arrows', false,...
        'cyl_half_length',0.25,...
        'cyl_radius',0.125};

Q_path_disp = Q_path(:,:,1:4:length(Q_path));
p_path_disp = p_path(:,1:4:length(Q_path));
N_disp = length(Q_path_disp);

% for i = 20
% for i = 45
% for i = 60
for i = 70
h_fig = diagrams.setup([4,4]); hold on

% diagrams.circle(Z*ez, ez, D1);
% diagrams.circle(Z*ez, ez, D2, lineStyle=":");
% diagrams.utils.plot3_mat(p_path, color = diagrams.colors.red, lineWidth=3);

draw_arc(Z*ez + r_outer*rot(ez, deg2rad(30))*ex, ez, ez, deg2rad(120), color=diagrams.colors.red);
diagrams.line(Z*ez, Z*ez + r_outer*rot(ez, deg2rad(30))*ex, color=diagrams.colors.red)
diagrams.line(Z*ez, Z*ez + r_outer*rot(ez, deg2rad(150))*ex, color=diagrams.colors.red)
draw_arc(Z*ez + r_outer*rot(ez, deg2rad(150))*ex, ez, ez, deg2rad(240), lineStyle=":");
diagrams.circle(Z*ez, ez, D2, lineStyle=":");

if i < 55
    diagrams.robot_plot(kin, Q_path_disp(:,1,i), rob_opts{:}, link_color=diagrams.colors.blue);
    % diagrams.robot_plot(kin, Q_path_disp(:,3,i), rob_opts{:}, link_color=diagrams.colors.red);
    diagrams.robot_plot(kin, Q_path_disp(:,5,i), rob_opts{:}, link_color=diagrams.colors.green);
    % diagrams.robot_plot(kin, Q_path_disp(:,7,i), rob_opts{:}, link_color=diagrams.colors.black);
else
    diagrams.robot_plot(kin, Q_path_disp(:,1,i), rob_opts{:}, link_color=diagrams.colors.green);
    % diagrams.robot_plot(kin, Q_path_disp(:,3,i), rob_opts{:}, link_color=diagrams.colors.red);
    diagrams.robot_plot(kin, Q_path_disp(:,5,i), rob_opts{:}, link_color=diagrams.colors.blue);
    % diagrams.robot_plot(kin, Q_path_disp(:,7,i), rob_opts{:}, link_color=diagrams.colors.black);
end

% diagrams.arrow(zv, p_path_disp(:,i));
campos([-18.9291  -24.6688   18.7153]);
camva(11.1433);
camtarget([0         0    0.7629]);

diagrams.redraw(); hold off
end
%%
diagrams.save(gcf, "IRB_3D_"+i)
%% GIF
filename = "6640_singular_IK.gif";
clear im

rob_opts = {'show_base_label', false,...
        'show_task_label', false,...
        'show_base_frame', false,...
        'show_task_frame', false,...
        'show_arrow_labels', false,...
        'show_joint_labels', false,...
        'show_arrows', false,...
        'cyl_half_length',0.25,...
        'cyl_radius',0.125};

Q_path_disp = Q_path(:,:,1:4:length(Q_path));
p_path_disp = p_path(:,1:4:length(Q_path));
N_disp = length(Q_path_disp);

% for i = 1:N_disp
for i = 35
h_fig = diagrams.setup([4,4]); hold on

diagrams.circle(Z*ez, ez, D1);
diagrams.circle(Z*ez, ez, D2, lineStyle=":");

diagrams.utils.plot3_mat(p_path, color = diagrams.colors.red);

diagrams.robot_plot(kin, Q_path_disp(:,1,i), rob_opts{:}, link_color=diagrams.colors.blue);
diagrams.robot_plot(kin, Q_path_disp(:,3,i), rob_opts{:}, link_color=diagrams.colors.red);
diagrams.robot_plot(kin, Q_path_disp(:,5,i), rob_opts{:}, link_color=diagrams.colors.green);
diagrams.robot_plot(kin, Q_path_disp(:,7,i), rob_opts{:}, link_color=diagrams.colors.black);

% diagrams.arrow(zv, p_path_disp(:,i));
campos([-18.9291  -24.6688   18.7153]);
camva(11.1433);
camtarget([0         0    0.7629]);

diagrams.redraw(); hold off

frame = getframe(h_fig);
im{i} = frame2im(frame);
end

%%
for idx = 1:length(im)
    [A, map] = rgb2ind(im{idx}, 256);
    if idx == 1
        imwrite(A, map, filename, "gif", "LoopCount", Inf, "DelayTime", 1/30);
    else
        imwrite(A, map, filename, "gif", "WriteMode", "append", "DelayTime", 1/30);
    end
end

%%
[R_t, p_t] = fwdkin(kin, Q_path(:,1,225))


%%
q_path_video = q_path_D(:, 1:end-1);
path_name = "D";
%% Video part 1: graph of q5
h_fig = figure(10);
% tiledlayout(1,1,'TileSpacing','none','Padding','compact');
% nexttile
figure_size = 2*[3.5 3];
set(h_fig, "Units", "inches")
pos_old = h_fig.OuterPosition;
if ~all(pos_old(3:4) == figure_size)
set(h_fig, "OuterPosition", [pos_old(1:2)-figure_size+pos_old(3:4) figure_size])
end
set(h_fig, "Units", "pixels")
findfigs
set(h_fig, 'renderer', 'painters')

q5_disp = squeeze(Q_path(5,:,:));

plot(q5_disp(:,1:2:end)', 'k.', MarkerSize =2); hold on
path_plot = plot((1:120)/2+0.5, q_path_video(5, 1:120), 'k', LineWidth=2);
hold off

xlabel('$k$', Interpreter="latex");
ylabel('$q_5$', Interpreter="latex");

ylim([-pi, pi]);
yticks([-pi, -pi/2, 0, pi/2, pi]);
yticklabels({'$-\pi$', '$-\frac{\pi}{2}$', '$0$', '$\frac{\pi}{2}$', '$\pi$'})


xaxisproperties= get(gca, 'XAxis');
xaxisproperties.TickLabelInterpreter = 'latex';
yaxisproperties= get(gca, 'YAxis');
yaxisproperties.TickLabelInterpreter = 'latex';

fontsize(gca, 2*10,"points")
%%
for i = 1:width(q_path_video)
    set(path_plot, XData = (1:i)/2+0.5, YData= q_path_video(5, 1:i))
    drawnow
    filename = sprintf('IRB_q5_plot_%s_%03.0f.png', path_name, i);
    % pause(1/30)
    saveas(gcf, filename) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
%% Video part 2: Robot in 3D

rob_opts = {'show_base_label', false,...
        'show_task_label', false,...
        'show_base_frame', false,...
        'show_task_frame', false,...
        'show_arrow_labels', false,...
        'show_joint_labels', false,...
        'show_arrows', false,...
        'cyl_half_length',0.25,...
        'cyl_radius',0.125};

for i = 1:width(q_path_video)
h_fig = diagrams.setup([6,6]); hold on

draw_arc(Z*ez + r_outer*rot(ez, deg2rad(30))*ex, ez, ez, deg2rad(120), color=diagrams.colors.red);
diagrams.line(Z*ez, Z*ez + r_outer*rot(ez, deg2rad(30))*ex, color=diagrams.colors.red);
diagrams.line(Z*ez, Z*ez + r_outer*rot(ez, deg2rad(150))*ex, color=diagrams.colors.red);
draw_arc(Z*ez + r_outer*rot(ez, deg2rad(150))*ex, ez, ez, deg2rad(240), lineStyle=":");
diagrams.circle(Z*ez, ez, D2, lineStyle=":");

diagrams.robot_plot(kin, q_path_video(:,i), rob_opts{:}, link_color=diagrams.colors.blue);

% Desired pose
diagrams.utils.plot3_mat(p_path(:,i), marker='o', color = diagrams.colors.red);

campos([-18.9291  -24.6688   18.7153]);
camva(11.1433);
camtarget([0         0    0.7629]);

diagrams.redraw(); hold off

drawnow
filename = sprintf('IRB_3D_plot_%s_%03.0f.png', path_name, i);
saveas(gcf, filename) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

%%

function draw_arc(tail, k, center, theta, varargin)
    N = 100;
    delta_theta = 2*pi / N;
    angles = [0:delta_theta:theta theta];
    C = NaN([3 length(angles)]);
    for i = 1:length(angles)
        C(:,i) = center+diagrams.rot(k, angles(i))*(tail-center);
    end
    diagrams.utils.plot3_mat(C, varargin{:});
end