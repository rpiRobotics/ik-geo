function plot_planar_bot_dots(L1, L2, q_1, q_2, varargin)

ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zv = [0;0;0];

p_12 = ex*L1;
p_2T = ex*L2;

p_02 = rot(ez, q_1)*p_12;
p_0T = p_02 + rot(ez, q_1+q_2)*p_2T;

diagrams.line(zv, p_02, varargin{:});
diagrams.line(p_02, p_0T, varargin{:});
diagrams.dot(zv, varargin{:});
diagrams.dot(p_02, varargin{:});
diagrams.dot(p_0T, varargin{:});


    
end