q_1 = pi/6;
q_2 = pi/3;

ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];

p_12 = ex*1.5;
p_2T = ex;

p_02 = rot(ez, q_1)*p_12;
p_0T = p_02 + rot(ez, q_1+q_2)*p_2T;


diagrams.setup([2 3]); hold on
view(2);

diagrams.line(zv, p_02);
diagrams.line(p_02, p_0T);
diagrams.dot(zv);
diagrams.dot(p_02);
diagrams.dot(p_0T);

diagrams.text(zv, "$\mathcal O_0 = \mathcal O_1$");
diagrams.text(p_02, "$\mathcal O_2$");
diagrams.text(p_0T, "$\mathcal O_T$");

diagrams.line(zv, 0.5*ex, LineStyle=":");
diagrams.angle_arc(0.25*ex, ez, zv, q_1);

diagrams.line(p_02, p_02 + 0.5*normalize(p_02), LineStyle=":");
diagrams.angle_arc(p_02 + 0.25*normalize(p_02), ez, p_02, q_2);

diagrams.text(rot(ez, q_1/2)*0.25*ex, "$q_1$", verticalAlign="middle");
diagrams.text(p_02 + rot(ez, q_2/2)*0.25*normalize(p_02), "$q_2$", verticalAlign="bottom");


diagrams.redraw(); hold off

function e = normalize(v)
    e = v / norm(v);
end