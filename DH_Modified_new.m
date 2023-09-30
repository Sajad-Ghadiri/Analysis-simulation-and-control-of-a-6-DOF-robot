%% clear
clear all;
clc;

%% T matrixes
P = sym("P" , [1 7]);
Theta = sym("Th",[1 6]);

T1_0 = transformation(sym(0), sym(0) , P(1), Theta(1));
T2_1 = transformation(-sym(pi/2), sym(0), P(2), -sym(pi/2)+Theta(2));
T3_2 = transformation(sym(0), P(3), sym(0), sym(pi/2)+Theta(3));
T4_3 = transformation(sym(pi/2), sym(0), P(4), Theta(4));
T5_4 = transformation(-sym(pi/2), sym(0), sym(0), Theta(5));
T6_5 = transformation(sym(pi/2), sym(0), sym(0), Theta(6));
T7_6 = [sym(1) sym(0) sym(0) sym(0); sym(0) sym(1) sym(0) sym(0); sym(0) sym(0) sym(1) P(7); sym(0) sym(0) sym(0) sym(1)];
T2_0 = T1_0*T2_1;
T3_0 = T1_0*T2_1*T3_2;
T4_0 = T1_0*T2_1*T3_2*T4_3;
T5_0 = T1_0*T2_1*T3_2*T4_3*T5_4;
T6_0 = T1_0*T2_1*T3_2*T4_3*T5_4*T6_5;
T7_0 = T1_0*T2_1*T3_2*T4_3*T5_4*T6_5*T7_6;











%% Modified DH function
function DH_Modified = transformation(alphai_1, ai_1, di, thetai)
DH_Modified = [cos(thetai) -sin(thetai) sym(0) ai_1; sin(thetai)*cos(alphai_1) cos(thetai)*cos(alphai_1) -sin(alphai_1) -sin(alphai_1)*di; sin(thetai)*sin(alphai_1) cos(thetai)*sin(alphai_1) cos(alphai_1) cos(alphai_1)*di; sym(0) sym(0) sym(0) sym(1)];
end
