%% clear
clear all;
clc;
%% 
jacobian_modified_firstmethod;
m = [7.388 6.323 2.566 0.174 1.453 0.120];
D = 0;
I1 = [-0.08 0.08 0.99;0.87 -0.49 0.1;0.49 0.87 -0.03];
I2 = [-0.2 0.51 0.84;0.92 -0.2 0.34;0.34 0.84 -0.43];
I3 = [0.29 0.3 0.91;-0.64 -0.65 0.41;0.72 -0.7 0];
I4 = [0.73 -0.69 0;0 0 -1;0.69 0.73 0];
I5 = [0.02 -0.03 1;-0.44 -0.9 -0.02;0.9 -0.44 -0.03];
I6 = [-0.40 0.18 0.90; 0.79 -0.42 0.44; 0.45 0.89 0.03];
I = [I1 I2 I3 I4 I5 I6];
Jv = [Jv1 Jv2 Jv3 Jv4 Jv5 Jv6];
Jw = [Jw1 Jw2 Jw3 Jw4 Jw5 Jw6];
ct_i = 1;
ct_j = 1;
%%D
for i=1:6
    D = D + (m(i)*transpose(Jv(:, ct_j:ct_j+5))*Jv(:, ct_j:ct_j+5)+transpose(Jw(:, ct_j:ct_j+5))*I(:,ct_i:ct_i+2)*Jw(:, ct_j:ct_j+5));
    ct_i = ct_i + 3;
    ct_j = ct_j + 6;
end
%%C
% C = sym(zeros(6,36));
C = sym(zeros(6, 6));
% ct = 0;
sum = 0;
for k=1:6
    for i=1:6
        for j=1:6
%          C(i, j+ct) = 1/2*(diff(D(k,j), Theta(i))+diff(D(k,i), Theta(j))-diff(D(i,j),Theta(k)));  
         sum = sum + 1/2*(diff(D(k,j), Theta(i))+diff(D(k,i), Theta(j))-diff(D(i,j),Theta(k)));
        end
        C(k, i) = sum;
        sum = 0;
    end
end


%% G

h1 = P(1)/2;
h2 = P(1);
h3 = P(1)+P(3)*cos(Theta(2)) ;
h4 = P(1)+(P(3)+(P(4)/2)*cos(Theta(3))*cos(Theta(2)));
h5 = P(1)+(P(3)+(P(4)*cos(Theta(3)))*cos(Theta(2)));
h6 = P(1)+(P(3)+((P(4)+P(6)*cos(Theta(5)))*cos(Theta(3)))*cos(Theta(2)));
h = [h1 h2 h3 h4 h5 h6];
P = 0;
g = 9.8;
for i=1:6
    P = P+m(i)*g*h(i);
end

phi_1 = diff(P, Theta(1));
phi_2 = diff(P, Theta(2));
phi_3 = diff(P, Theta(3));
phi_4 = diff(P, Theta(4));
phi_5 = diff(P, Theta(5));
phi_6 = diff(P, Theta(6));
G = [phi_1 phi_2 phi_3 phi_4 phi_5 phi_6]';











