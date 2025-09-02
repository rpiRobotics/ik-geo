q_1 = pi/6;
q_2 = pi/3;
q_3 = -pi/6;

ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];

p_12 = ex*1.5;
p_23 = ex;
p_3T = ex;

p_02 = rot(ez, q_1)*p_12;
p_03 = p_02 + rot(ez, q_1+q_2)*p_23;
p_0T = p_03 + rot(ez, q_1+q_2+q_3)*p_3T;


diagrams.setup([2 3]); hold on
view(2);

diagrams.line(zv, p_02);
diagrams.line(p_02, p_03);
diagrams.line(p_03, p_0T);

diagrams.dot(zv);
diagrams.dot(p_02);
diagrams.dot(p_03);
diagrams.dot(p_0T);

diagrams.redraw(); hold off
