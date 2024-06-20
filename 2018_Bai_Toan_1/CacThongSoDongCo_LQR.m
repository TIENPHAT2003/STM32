clear all;
close all;
clc;

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
Jm = 10^-2; %moment quan tinh cua dong co
Jw = m*R^2/2;
J_psi = M*L^2/3;
J_phi = M*(W^2+D^2)/12;
Rm = 13; %Dien tro dong co DC
Kb = 0.468; %he so emf cua dong co
Kt = 0.317; %Momen xoan cua dong co DC
n = 33.64; %Ty so giam toc
g = 9.81; %Gia toc trong truong
alpha = n*Kt/Rm; beta=n*Kt*Kb/Rm+fm; a =alpha;
T=0.01;

% Chọn thông số ban đầu
x1_init = 0.001; x2_init = -0.0012; x4_init = 0.002; x5_init = -0.002; x7_init = 0.002; x8_init=-0.0014;