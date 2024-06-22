clear all;
close all; 
clc;

syms m M R Jm L n a beta fw g W J_psi Jw vl vr J_phi x1 x2 x3 x4 x5 x6 x7 x8 x9
f1 = ((2*m+M)*R^2+2*Jw+2*n^2*Jm)*x3+(M*R*L*cos(x4)-2*n^2*Jm)*x6-M*L*R*x5^2*sin(x4)-a*(vl+vr)+2*(beta+fw)*x2-2*beta*x5                          
f2 = (M*L*R*cos(x4)-2*n^2*Jm)*x3+(M*L^2+J_psi+2*n^2*Jm)*x6-M*g*L*sin(x4)-M*L^2*x8^2*sin(x4)*cos(x4)+a*(vl+vr)-2*beta*x2+2*beta*x5              
f3 = (0.5*m*W^2+J_phi+W^2/2/R^2*(Jw+n^2*Jm)+M*L^2*(sin(x4))^2)*x9+2*M*L^2*x5*x8*sin(x4)*cos(x4)-W/2/R*a*(vr-vl)+W^2/2/R^2*(beta+fw)*x8         

[x3,x6,x9] = solve(f1,f2,f3,x3,x6,x9)                                                                                                          

%% Rút gọn bậc
syms m M R Jm L n a beta fw g W J_psi Jw vl vr J_phi x1 x2 x3 x4 x5 x6 x7 x8 x9
y1 = x2;
y2 = (J_psi*a*vl+J_psi*a*vr-2*J_psi*beta*x2+2*J_psi*beta*x5-2*J_psi*fw*x2+L^2*M*a*vl+L^2*M*a*vr-2*L^2*M*beta*x2+2*L^2*M*beta*x5-2*L^2*M*fw*x2-4*Jm*fw*n^2*x2+L^3*M^2*R*x5^2*sin(x4)-2*L*M*R*beta*x2*cos(x4)+2*L*M*R*beta*x5*cos(x4)-L^3*M^2*R*x8^2*cos(x4)^2*sin(x4)-L^2*M^2*R*g*cos(x4)*sin(x4)+J_psi*L*M*R*x5^2*sin(x4)+2*Jm*L*M*g*n^2*sin(x4)+L*M*R*a*vl*cos(x4)+L*M*R*a*vr*cos(x4)+2*Jm*L^2*M*n^2*x8^2*cos(x4)*sin(x4)+2*Jm*L*M*R*n^2*x5^2*sin(x4))/(2*J_psi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(x4)^2 + 4*Jm*L*M*R*n^2*cos(x4)) 
y3 = x5;
y4 = -(2*Jw*a*vl + 2*Jw*a*vr - 4*Jw*beta*x2 + 4*Jw*beta*x5 + M*R^2*a*vl + M*R^2*a*vr - 2*M*R^2*beta*x2 + 2*M*R^2*beta*x5 + 2*R^2*a*m*vl + 2*R^2*a*m*vr + 4*Jm*fw*n^2*x2 - 4*R^2*beta*m*x2 + 4*R^2*beta*m*x5 - L*M^2*R^2*g*sin(x4) - 2*Jw*L*M*g*sin(x4) - 2*L*M*R*beta*x2*cos(x4) + 2*L*M*R*beta*x5*cos(x4) - 2*L*M*R*fw*x2*cos(x4) + L^2*M^2*R^2*x5^2*cos(x4)*sin(x4) - L^2*M^2*R^2*x8^2*cos(x4)*sin(x4) - 2*Jw*L^2*M*x8^2*cos(x4)*sin(x4) - 2*Jm*L*M*g*n^2*sin(x4) - 2*L*M*R^2*g*m*sin(x4) + L*M*R*a*vl*cos(x4) + L*M*R*a*vr*cos(x4) - 2*Jm*L^2*M*n^2*x8^2*cos(x4)*sin(x4) - 2*L^2*M*R^2*m*x8^2*cos(x4)*sin(x4) - 2*Jm*L*M*R*n^2*x5^2*sin(x4))/(2*J_psi*Jw + L^2*M^2*R^2 + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 - L^2*M^2*R^2*cos(x4)^2 + 4*Jm*L*M*R*n^2*cos(x4))
y5 = x8;
y6 = -(W^2*beta*x8 + W^2*fw*x8 + R*W*a*vl - R*W*a*vr + 4*L^2*M*R^2*x5*x8*cos(x4)*sin(x4))/(2*J_phi*R^2 + Jw*W^2 + Jm*W^2*n^2 + R^2*W^2*m + 2*L^2*M*R^2*sin(x4)^2)

%% Thông số hệ thống xe 2 bánh tự cân bằng dùng LQR
m = 0.0345; %Khoi luong banh xe
M = 0.875; %Khoi luong robot
R = 0.0325; %ban kinh ban xe
W = 0.225; %Chieu rong robot
D = 0.084; %Chieu sau robot
H = 0.132; %Chieu cao robot
L = 0.091; %khoang cach tu trong tam den truc banh xe
fw = 0.18; %He so ma sat giua banh xe voi mat phang
fm = 0.002; %he so ma sat giua dong co va robot
Jm = 0.000082; %moment quan tinh cua dong co
Jw = m*R^2/2;
J_psi = M*L^2/3;
J_phi = M*(W^2+D^2)/12;
Rm = 13; %Dien tro dong co DC
Kb = 1.91; %he so emf cua dong co
Kt = 0.216 ; %Momen xoan cua dong co DC
n = 33.64; %Ty so giam toc
g = 9.81; %Gia toc trong truong
alpha = n*Kt/Rm; beta=n*Kt*Kb/Rm+fm; a =alpha;
T=0.01;

%% Tính ma trận A
% A =   [ diff(y1,x1) diff(y1,x2) diff(y1,x4) diff(y1,x5) diff(y1,x7) diff(y1,x8);
%         diff(y2,x1) diff(y2,x2) diff(y2,x4) diff(y2,x5) diff(y2,x7) diff(y2,x8);
%         diff(y3,x1) diff(y3,x2) diff(y3,x4) diff(y3,x5) diff(y3,x7) diff(y3,x8);
%         diff(y4,x1) diff(y4,x2) diff(y4,x4) diff(y4,x5) diff(y4,x7) diff(y4,x8);
%         diff(y5,x1) diff(y5,x2) diff(y5,x4) diff(y5,x5) diff(y5,x7) diff(y5,x8);
%         diff(y6,x1) diff(y6,x2) diff(y6,x4) diff(y6,x5) diff(y6,x7) diff(y6,x8)]   
%Ma trận A tại điểm cân bằng 
A = [0,                                                                                                                                                                                                                                                                1,                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                    0, 0,                                                                    0;
0, -(2*J_psi*beta + 2*J_psi*fw + 2*L^2*M*beta + 2*L^2*M*fw + 4*Jm*fw*n^2 + 2*L*M*R*beta)/(2*J_psi*Jw + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 + 4*Jm*L*M*R*n^2),                             -(R*g*L^2*M^2 - 2*Jm*g*L*M*n^2)/(2*J_psi*Jw + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 + 4*Jm*L*M*R*n^2),              (2*M*beta*L^2 + 2*M*R*beta*L + 2*J_psi*beta)/(2*J_psi*Jw + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 + 4*Jm*L*M*R*n^2), 0,                                                                    0;
0,                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                    1, 0,                                                                    0;
0,   (4*Jw*beta + 2*M*R^2*beta - 4*Jm*fw*n^2 + 4*R^2*beta*m + 2*L*M*R*beta + 2*L*M*R*fw)/(2*J_psi*Jw + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 + 4*Jm*L*M*R*n^2), (L*g*M^2*R^2 + 2*L*g*m*M*R^2 + 2*Jm*L*g*M*n^2 + 2*Jw*L*g*M)/(2*J_psi*Jw + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 + 4*Jm*L*M*R*n^2), -(4*Jw*beta + 2*M*R^2*beta + 4*R^2*beta*m + 2*L*M*R*beta)/(2*J_psi*Jw + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 + 4*Jm*L*M*R*n^2), 0,                                                                    0;
0,                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                    0, 0,                                                                    1;
0,                                                                                                                                                                                                                                                                0,                                                                                                                                                                                                                                      0,                                                                                                                                                                                                                                    0, 0, -(W^2*beta + W^2*fw)/(m*R^2*W^2 + 2*J_phi*R^2 + Jm*W^2*n^2 + Jw*W^2)]

%% Tính ma trận B
% B = [ diff(y1,vl) diff(y1,vr);
%       diff(y2,vl) diff(y2,vr);
%       diff(y3,vl) diff(y3,vr);
%       diff(y4,vl) diff(y4,vr);
%       diff(y5,vl) diff(y5,vr);
%       diff(y6,vl) diff(y6,vr)]
%                               % Ma trận B tại điểm cân bằng
B =[                                                                                                                                                                                                                   0,                                                                                                                                                                                                                    0;
            (M*a*L^2 + M*R*a*L + J_psi*a)/(2*J_psi*Jw + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 + 4*Jm*L*M*R*n^2),             (M*a*L^2 + M*R*a*L + J_psi*a)/(2*J_psi*Jw + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 + 4*Jm*L*M*R*n^2);
                                                                                                                                                                                                                   0,                                                                                                                                                                                                                    0;
-(2*Jw*a + M*R^2*a + 2*R^2*a*m + L*M*R*a)/(2*J_psi*Jw + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 + 4*Jm*L*M*R*n^2), -(2*Jw*a + M*R^2*a + 2*R^2*a*m + L*M*R*a)/(2*J_psi*Jw + 2*Jw*L^2*M + J_psi*M*R^2 + 2*J_psi*Jm*n^2 + 4*Jm*Jw*n^2 + 2*J_psi*R^2*m + 2*Jm*L^2*M*n^2 + 2*Jm*M*R^2*n^2 + 2*L^2*M*R^2*m + 4*Jm*R^2*m*n^2 + 4*Jm*L*M*R*n^2);
                                                                                                                                                                                                                   0,                                                                                                                                                                                                                    0;
                                                                                                                                                            -(R*W*a)/(m*R^2*W^2 + 2*J_phi*R^2 + Jm*W^2*n^2 + Jw*W^2),                                                                                                                                                              (R*W*a)/(m*R^2*W^2 + 2*J_phi*R^2 + Jm*W^2*n^2 + Jw*W^2)]
 

%% Tính thông số K của LQR
R__ = [100 0; 0 100] %Chọn R__ vì ở trên đã có R là bán kính bánh xe rồi
Q = [ 1 0 0 0 0 0;
      0 1 0 0 0 0;
      0 0 1 0 0 0;
      0 0 0 1 0 0;
      0 0 0 0 1 0; 
      0 0 0 0 0 1]
 K = lqr(A,B,Q,R__)

 % Chọn thông số ban đầu

x1_init = 0.001; x2_init = -0.0012; x4_init = 0.002; x5_init = -0.002; x7_init = 0.002; x8_init=-0.0014;





