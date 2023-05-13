
q = [-1.1527587103475838 
    2.92902113037483 
    -0.9524425097091209 
    -1.8095571193251818 
    0.7821349481829708 
    0.6035317069931163 ]

kin = define_kuka_kr30;

[R, T] = fwdkin(kin, q)

% From IKfast:
% R = 
%     0.5743211080994187 -0.5775810071795288 0.580133988779262
%     -0.39396467507922095 -0.8162106151496654 -0.4226015457931132
%     0.7175981463055627 0.014156689672593564 -0.6963134987606762
% 
% T =
%     -0.17161960826424538
%     -0.6528587966300649
%     1.1176901423647452

function kin = define_kuka_kr30
    % https://ballardintl.com/wp-content/uploads/2021/01/Kuka-KR30_KR60-Specs.pdf
    % https://github.com/rdiankov/collada_robots/blob/98c0760b8cdff9c4cd754752184421f21ff19946/kuka-kr30l16.zae
    
    % Careful:
    % Joint directions in openrave are different than in kuka manual

    zv = [0;0;0];
    ex = [1;0;0];
    ey = [0;1;0];
    ez = [0;0;1];
    kin.H = [-ez -ey -ey -ex -ey -ex];
    kin.P = [zv 0.35*ex+0.815*ez 1.2*ez 0.145*ez+1.545*ex zv zv 0.158*ex];
    kin.joint_type = zeros([6 1]);
end