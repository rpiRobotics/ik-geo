L1 = 1;
L2 = 1;

ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];

r_outer = L1 + L2;

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


diagrams.setup([2 3]); hold on
view(2);

Q = planar_bot_IK(p_path(:,150), L1, L2);
% Q = planar_bot_IK(zv, L1, L2);
plot_planar_bot_dots(L1, L2, Q(1,1), Q(2,1));
plot_planar_bot_dots(L1, L2, Q(1,2), Q(2,2));

diagrams.circle(zv, ez, r_outer, LineStyle=":");

diagrams.utils.plot3_mat(p_path);
for i = 1:25:4*N
    diagrams.arrow(p_path(:,i), p_path(:,i)+e_sing_path(:,i)*0.2);
end

diagrams.redraw(); hold off
%%
plot(p_path(1:2,:)', '-x')


%%  IK on the whole path

q1_path = NaN([2 length(p_path)]);
q2_path = NaN([2 length(p_path)]);
is_LS_1 = NaN([2 length(p_path)]);
is_LS_2 = NaN([2 length(p_path)]);
for i = 1:length(p_path)
    % [Q, is_LS] = planar_bot_IK(p_path(:,i), L1, L2);
    [Q, is_LS] = planar_bot_IK_sing(p_path(:,i), L1, L2, e_sing_path(:,i));
    
    q1_path(:,i) = Q(1,:);
    q2_path(:,i) = Q(2,:);
    is_LS_1(:,i) = is_LS(1,:);
    is_LS_2(:,i) = is_LS(2,:);
end
%%
plot(1:length(p_path), q1_path', '.')
xlabel("\lambda")
ylabel("q_1")
%%
plot(1:length(p_path), q2_path', '.')
xlabel("\lambda")
ylabel("q_2")

%%
plot(is_LS_1', 'x')
%%
plot(is_LS_2', 'x')

%% Plot IK errors
i_soln = 1;

p_12 = ex*L1;
p_2T = ex*L2;

p_fk = NaN([3 length(p_path)]);
for i = 1:length(p_path)
    p_fk(:,i) = rot(ez, q1_path(i_soln, i)) *(p_12 + rot(ez, q2_path(i_soln, i)) * p_2T);
end

e = vecnorm(p_fk - p_path);
semilogy(e)

%% gif of robot moving
filename = "planar_singular_IK.gif";
clear im

q1_path_disp = q1_path(:,1:4:end);
q2_path_disp = q2_path(:,1:4:end);

N_disp = length(q1_path_disp);

for i = 1:N_disp
    h_fig = diagrams.setup([2 3]); hold on
    view(2);
    plot_planar_bot_dots(L1, L2, q1_path_disp(1, i), q2_path_disp(1, i), color=diagrams.colors.red);
    plot_planar_bot_dots(L1, L2, q1_path_disp(2, i), q2_path_disp(2, i), color=diagrams.colors.green);
    
    diagrams.circle(zv, ez, r_outer, LineStyle=":");
    
    diagrams.utils.plot3_mat(p_path);
    for j = 1:25:4*N
        diagrams.arrow(p_path(:,j), p_path(:,j)+e_sing_path(:,j)*0.2);
    end
    
    diagrams.redraw(); hold off
    frame = getframe(h_fig);
    im{i} = frame2im(frame);
end
%%
for idx = 1:length(im)
    [A, map] = rgb2ind(im{idx}, 256);
    if idx == 1
        imwrite(A, map, filename, "gif", "LoopCount", Inf, "DelayTime",1/30);
    else
        imwrite(A, map, filename, "gif", "WriteMode", "append", "DelayTime", 1/30);
    end
end