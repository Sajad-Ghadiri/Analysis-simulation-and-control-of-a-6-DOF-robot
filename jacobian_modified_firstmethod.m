%% clear
clear all;
clc;
%% alternative approach
DH_Modified_new;
%%end effector jacobian matrix

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
jacobian_endeffector = [cross(z1, o7-o1) cross(z2, o7-o2) cross(z3, o7-o3) cross(z4, o7-o4) cross(z5, o7-o5) cross(z6, o7-o6); z1 z2 z3 z4 z5 z6];

%%joints jacobian
Tc1_1 = [1 0 0 0; 0 1 0 0; 0 0 1 -160; 0 0 0 1];
Tc2_2 = [1 0 0 0; 0 1 0 0; 0 0 1 21.5; 0 0 0 1];
Tc3_3 = [1 0 0 0; 0 1 0 0; 0 0 1 85.5; 0 0 0 1];
Tc4_4 = [1 0 0 0; 0 1 0 0; 0 0 1 -112.5; 0 0 0 1];
Tc5_5 = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Tc6_6 = [1 0 0 0; 0 1 0 0; 0 0 1 32.5; 0 0 0 1];
%%% center of mass according to frame0
Tc1_0 = T1_0 * Tc1_1;
Tc2_0 = T2_0 * Tc2_2;
Tc3_0 = T3_0 * Tc3_3;
Tc4_0 = T4_0 * Tc4_4;
Tc5_0 = T5_0 * Tc5_5;
Tc6_0 = T6_0 * Tc6_6;

%%% third column of each T
zc1 = Tc1_0(1:3, 3);
zc2 = Tc2_0(1:3, 3);
zc3 = Tc3_0(1:3, 3);
zc4 = Tc4_0(1:3, 3);
zc5 = Tc5_0(1:3, 3);
zc6 = Tc6_0(1:3, 3);

oc1 = Tc1_0(1:3, 4);
oc2 = Tc2_0(1:3, 4);
oc3 = Tc3_0(1:3, 4);
oc4 = Tc4_0(1:3, 4);
oc5 = Tc5_0(1:3, 4);
oc6 = Tc6_0(1:3, 4);

%%%  Jv and Jw
Jv1 = [cross(zc1, oc1-o1) zeros(3,5)];
Jw1 = [zc1 zeros(3,5)];
Jv2 = [cross(zc1, oc2-o1) cross(zc2, oc2-o2) zeros(3,4)];
Jw2 = [zc1 zc2 zeros(3,4)];
Jv3 = [cross(zc1, oc3-o1) cross(zc2, oc3-o2) cross(zc3, oc3-o3) zeros(3,3)];
Jw3 = [zc1 zc2 zc3 zeros(3,3)];
Jv4 = [cross(zc1, oc4-o1) cross(zc2, oc4-o2) cross(zc3, oc4-o3) cross(zc4, oc4-o4) zeros(3,2)];
Jw4 = [zc1 zc2 zc3 zc4 zeros(3,2)];
Jv5 = [cross(zc1, oc5-o1) cross(zc2, oc5-o2) cross(zc3, oc5-o3) cross(zc4, oc5-o4) cross(zc5, oc5-o5) zeros(3,1)];
Jw5 = [zc1 zc2 zc3 zc4 zc5 zeros(3,1)];
Jv6 = [cross(zc1, oc6-o1) cross(zc2, oc6-o2) cross(zc3, oc6-o3) cross(zc4, oc6-o4) cross(zc5, oc6-o5) cross(zc6, oc6-o6)];
Jw6 = [zc1 zc2 zc3 zc4 zc5 zc6];





