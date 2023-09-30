%% clear
clear all;
clc;

%% T matrixes
syms Xee Yee Zee;
syms p1 p2 p3 p4 p5 p6 p7;
syms theta1 theta2 theta3 theta4 theta5 theta6;
%Desired Position for end effector
Xee = 0;
Yee = 27.36;
Zee = 730;
p1 = 320;
p2 = 35;
p3 = 225;
p4 = 225;
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
%% inverse kinematic
%Find Three Angles(theta1, theta2, theta3) from origin of joint4
eq1 = T4_0(1, 4) == Xee;
eq2 = T4_0(2, 4) == Yee;
eq3 = T4_0(3, 4) == Zee;
[angle1, angle2, angle3] = vpasolve([eq1, eq2, eq3],[theta1, theta2, theta3]);

angle1 = mod(mod(angle1, 2 * vpa(pi)), 2 * vpa(pi));
angle2 = mod(mod(angle2, 2 * vpa(pi)), 2 * vpa(pi));
angle3 = mod(mod(angle3, 2 * vpa(pi)), 2 * vpa(pi));

T6_3 = T4_3 * T5_4 * T6_5;
R6_3 = T6_3(1:3,1:3);

R3_0 = T3_0(1:3, 1:3);
R3_0 = subs(R3_0,'theta1', angle1);
R3_0 = subs(R3_0,'theta2', angle2);
R3_0 = subs(R3_0,'theta3', angle3);
R3_0 = vpa(R3_0);

angle = euler(R3_0, R6_3)
%% Euler
function Euler_angles = euler(R3_0, R6_3)
syms psii phi theta;
R_z_phi =[cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];
R_y_theta = [cos(theta) 0 sin(theta); 0 1 0;-sin(theta) 0 cos(theta)];
R_z_psi = [cos(psii) -sin(psii) 0;sin(psii) cos(psii) 0;0 0 1];
R_zyz = R_z_phi * R_y_theta * R_z_psi;
R6_0 = R3_0 * R6_3;
theta = solve(R_zyz(3,3) - R6_0(3,3), theta);
R_zyz = subs(R_zyz, 'theta', theta);
psii= solve(R_zyz(3,2) - R6_0(3,2), psii);
phi = solve(R_zyz(2,3) - R6_0(2,3), phi);
Euler_angles = [phi theta psii];
end
%% Modified DH function
function DH_Modified = transformation(alphai_1, ai_1, di, thetai)
DH_Modified = [cos(thetai) -sin(thetai) 0 ai_1; sin(thetai)*cos(alphai_1) cos(thetai)*cos(alphai_1) -sin(alphai_1) -sin(alphai_1)*di; sin(thetai)*sin(alphai_1) cos(thetai)*sin(alphai_1) cos(alphai_1) cos(alphai_1)*di; sym(0) sym(0) sym(0) sym(1)];
end