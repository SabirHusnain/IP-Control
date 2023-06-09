clear; clc; close all;

%% Model
%%%
load('myNewMotor4.mat');
r=0.01; g=9.81;
M=0.770+0.032+0.019+0.032;
m=0.137;
l=0.175;
I=(1/12)*m*(2*l)^2;

M_1=M+m+(J_m/(r^2));
I_1=m*l;
C_1=K_m/(r*R_m);
C_2=((K_m^2)/((r^2)*R_m))+(B_m/(r^2));
C_3=m*g*l;
I_2=I+m*(l^2);
Z=-(M_1*I_2)+(I_1^2);

A=[0 1 0 0;
    -M_1*C_3/Z 0 0 I_1*C_2/Z;
    0 0 0 1;
    -C_3*I_1/Z 0 0 C_2*I_2/Z];
B=[0; -I_1*C_1/Z; 0; -C_1*I_2/Z];
C=[1 0 0 0;
    0 0 1 0];
D=zeros(2,1);

%% Control Design & Implimentation
%%%
[num den]=ss2tf(A,B,C,D);
G=tf(num(1,:),den);
Num_tf=G.Numerator{1,1};
Den_tf=G.Denominator{1,1};
N1=Num_tf(3); N2=-Num_tf(4); N3=Num_tf(5);
D1=Den_tf(2); D2=-Den_tf(3); D3=-Den_tf(4);

K_D=(28-D1)/N1
K_I1=100000/N3
K_P=(D2/N1)+((N2/(N1^2))*(28-D1))+500/N1

K_I2=(N3*K_P-34000)/N2

K_I3=(5500-N3*K_D+N2*K_P+D3)/N1

% Gc1=tf([K_D K_P K_I1], [1 0]);
% Gc2=tf([K_D K_P K_I2], [1 0]);
Gc3=tf([K_D K_P K_I3], [1 0]);

% sys_Loop1=feedback(series(Gc1,G),1);
% sys_Loop2=feedback(series(Gc2,G),1);
sys_Loop3=feedback(series(Gc3,G),1);

sys_Robust=tf(10^5,[1 2.8*10 5*10^2 5.5*10^3 3.4*10^4 10^5]);

Gp=sys_Robust/sys_Loop3;

sys=series(Gp,sys_Loop3);

impulse(sys,2); grid on; grid minor;

%% Pre Filter Embedded Designing
Gp_z=c2d(Gp,0.025,'tustin');
Gp_z.Variable='z^-1';
Gp_z