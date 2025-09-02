L1 = 1;
L2 = 1;

ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];

r_outer = L1 + L2;



% Generate path
N = 1e5;
lambda = linspace(0, 2*pi, N);
p_path = NaN([3 N]);
for i = 1:N
    p_path(:,i) = ey + 1.001*rot(ez, lambda(i))*ex;
end


diagrams.setup([2 3]); hold on
view(2);


% Q = planar_bot_IK(p_path(:,1), L1, L2);
Q = planar_bot_IK(zv, L1, L2);
plot_planar_bot_dots(L1, L2, Q(1,1), Q(2,1));
plot_planar_bot_dots(L1, L2, Q(1,2), Q(2,2));

diagrams.circle(zv, ez, r_outer, LineStyle=":");


diagrams.utils.plot3_mat(p_path);

diagrams.redraw(); hold off

%% Perform IK over the whole path

R = rot(rand_normal_vec, rand_angle); % Global rotation

q1_path = NaN([2 N]);
q2_path = NaN([2 N]);

for i = 1:N
    Q = planar_bot_IK(p_path(:,i), L1, L2, R);
    q1_path(:,i) = Q(1,:);
    q2_path(:,i) = Q(2,:);
end
%%
plot(lambda, q1_path', '.')
xlabel("\lambda")
ylabel("q_1")
xlim([0 2*pi])

%%
i_soln = 1;

p_12 = ex*L1;
p_2T = ex*L2;

p_fk = NaN([3 N]);
for i = 1:N
    p_fk(:,i) = rot(ez, q1_path(i_soln, i)) *(p_12 + rot(ez, q2_path(i_soln, i)) * p_2T);
end

e = vecnorm(p_fk - p_path);
semilogy(e)