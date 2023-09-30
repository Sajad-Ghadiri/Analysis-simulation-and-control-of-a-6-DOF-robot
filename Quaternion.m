%% euler and quartner angles
DH_Modified_new;

syms K_x K_y K_z equal_theta
syms e1 e2 e3 e4
K = [K_x K_y K_z] ;
e1 = K_x * sin(equal_theta/2) ;
e2 = K_y * sin(equal_theta/2) ;
e3 = K_z * sin(equal_theta/2) ;
e4 = cos(equal_theta/2) ;

% As you can see e1, e2, e3 and e4 have below relation :
%  e1^2 + e2^2 + e3^2 + e4^2 == 1

%  it means that you can reach any orientation in space with these
%  parameters on the sphere with radius 1

R_e = [
        1 - 2*(e2^2 + e3^2) , 2*e1*e2 - e3*e4     , 2*e1*e3 + e2*e4    ;
        2*e1*e2 + e3*e4     , 1 - 2*(e1^2 + e3^2) , 2*(e2*e3 - e1*e4)  ;
        2*(e1*e3 - e2*e4)   , 2*(e2*e3 + e1*e4)   , 1- 2*(e1^2 + e2^2) ;] ;


%% show R6_0 elements

r11 = pretty(simplify(T6_0(1,1))) ;
r12 = pretty(simplify(T6_0(1,2))) ;
r13 = pretty(simplify(T6_0(1,3))) ;
r21 = pretty(simplify(T6_0(2,1))) ;
r22 = pretty(simplify(T6_0(2,2))) ;
r23 = pretty(simplify(T6_0(2,3))) ;
r31 = pretty(simplify(T6_0(3,1))) ;
r32 = pretty(simplify(T6_0(3,2))) ;
r33 = pretty(simplify(T6_0(3,3))) ;

%% 
