%% clear
clear all;
clc;

%% T matrixes
syms Xee Yee Zee;
syms p1 p2 p3 p4 p5 p6 p7;
syms theta1 theta2 theta3 theta4 theta5 theta6;
Xee = 200;
Yee = 123;
Zee = 835;
p1 = 320;
p2 = 35;
p3 = 225;
p4 = 225;
% p1 = 320; p2 = 35; p3 = 225; p4 = 225;
T1_0 = transformation(sym(0), sym(0) , p1, theta1);
T2_1 = transformation(-sym(pi/2), sym(0), p2, -sym(pi/2)+theta2);
T3_2 = transformation(sym(0), p3, sym(0), sym(pi/2)+theta3);
T4_3 = transformation(sym(pi/2), sym(0), p4, theta4);
T5_4 = transformation(-sym(pi/2), sym(0), sym(0), theta5);
T6_5 = transformation(sym(pi/2), sym(0), sym(0), theta6);
T7_6 = [1 0 0 0; 0 1 0 0; 0 0 1 p7; 0 0 0 1];
T2_0 = T1_0*T2_1;
T3_0 = T1_0*T2_1*T3_2;
T4_0 = T1_0*T2_1*T3_2*T4_3;
T5_0 = T1_0*T2_1*T3_2*T4_3*T5_4;
T6_0 = T1_0*T2_1*T3_2*T4_3*T5_4*T6_5;
T7_0 = T1_0*T2_1*T3_2*T4_3*T5_4*T6_5*T7_6;

ja = jacobian(6, T1_0, T2_0, T3_0, T4_0, T5_0, T6_0, T7_0);

%% inverse kinematic
%4th colmun of T4_3
P4_3 = T4_3(:,4);
P4_0 = T1_0 * T2_1 * T3_2 * P4_3;
F = T3_2 * P4_3;
%P4_0 = [cos(theta1) * g1 - sin(theta1) * g2; sin(theta1) * g1 + cos(theta1) * g2; g3; 1]
g1 = cos(theta2) * F(1,1) - sin(theta2) * F(2,1);
g2 = F(3,1) + p2;
g3 = -sin(theta2) * F(1,1) -cos(theta2) * F(2,1);
%r = g1^2 + g2^2 + g3^2;
%r = F(1,1)^2 + F(2,1)^2 + F(3,1)^2 + a1^2 + p2^2 + 2 * p2 * F(3,1) + 2 * a1 * (cos(theta1) * F(1,1) - sin(theta2) * F(2,1));
%r = (k1 * cos(theta2) + k2 * sin(theta2)) * 2 * a1 + k3;
%z = (k1 * sin(theta2) - k2 * cos(theta1)) * sin(alpha1) + k4;
k1 = F(1,1);
k2 = -F(2,1);
k3 = F(1,1)^2 + F(2,1)^2 + F(3,1)^2 + p2^2 + 2 * p2 * F(3,1);
k4 = F(3,1) + p2;
r = sqrt(Xee^2 + Yee^2 + Zee^2);
% a1 = 0 then r = k3
theta3 = solve(r-k3, theta3)
k1_new = subs(k1, 'theta3', theta3(2, 1));
k2_new = subs(k2, 'theta3', theta3(2, 1));
k3_new = subs(k3, 'theta3', theta3(2, 1));
k4_new = subs(k4, 'theta3', theta3(2, 1));
%Zee = (k1 * sin(theta2) - k2 * sin(theta1)) * sin(alpha1) + k4
theta2 = solve(k4_new - Zee - k1_new * sin(theta2) + k2_new * cos(theta2), theta2)
g1_new = subs(g1, {'theta2', 'theta3'}, {theta2(2, 1), theta3(2, 1)});
g2_new = subs(g2, {'theta2', 'theta3'}, {theta2(2, 1), theta3(2, 1)});
P4_0_new = subs(P4_0, {'theta2', 'theta3'}, {theta2(2, 1), theta3(2, 1)});
theta1 = solve(P4_0_new(1,1) - cos(theta1) * g1_new + sin(theta1) * g2_new, theta1)
%% Modified DH function
function DH_Modified = transformation(alphai_1, ai_1, di, thetai)
DH_Modified = [cos(thetai) -sin(thetai) 0 ai_1; sin(thetai)*cos(alphai_1) cos(thetai)*cos(alphai_1) -sin(alphai_1) -sin(alphai_1)*di; sin(thetai)*sin(alphai_1) cos(thetai)*sin(alphai_1) cos(alphai_1) cos(alphai_1)*di; sym(0) sym(0) sym(0) sym(1)];
end
%% end effector jacobian matrix
function jacobianOut = jacobian(i, T1_0, T2_0, T3_0, T4_0, T5_0, T6_0, T7_0)
z1 = T1_0(1:3, 3);
z2 = T2_0(1:3, 3);
z3 = T3_0(1:3, 3);
z4 = T4_0(1:3, 3);
z5 = T5_0(1:3, 3);
z6 = T6_0(1:3, 3);

o1 = T1_0(1:3, 4);
o2 = T2_0(1:3, 4);
o3 = T3_0(1:3, 4);
o4 = T4_0(1:3, 4);
o5 = T5_0(1:3, 4);
o6 = T6_0(1:3, 4);
o7 = T7_0(1:3, 4);

% if i==6
jacobianOut = [cross(z1, o7-o1) cross(z2, o7-o2) cross(z3, o7-o3) cross(z4, o7-o4) cross(z5, o7-o5) cross(z6, o7-o6); z1 z2 z3 z4 z5 z6];

end
