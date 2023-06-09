%% Model

clear; clc; close all;

load('myNewMotor3.mat');
K_t=K; K_e=K; J_eq=J; B_eq=B;
K_g=1;
r=0.01; g=9.81;
M=0.770+0.032+0.019+0.032; m=0.137; l=0.175;
C_a=(K_t*K_e)/(r*R*K_g)+B_eq/(r*K_g);
C_b=K_g*r*M+K_g*r*m+J_eq/(K_g*r);
z=((4/3)*l*C_b)-(K_g*r*m*l);

A=[0 1 0 0;
   C_b*g/z 0 0 C_a/z;
   0 0 0 1;
   -K_g*r*m*l*g/z 0 0 -(4*C_a*l)/(3*z)];
B=[0; -K_t/(R*z); 0; (4*K_t*l)/(3*R*z)];
C=[1 0 0 0;
   0 0 1 0];
D=zeros(2,1);

[num,den]=ss2tf(A,B,C,D);

G=tf(num(1,:),den)

%% Control Design

K_p=-101.648; K_i=-761.571; K_d=-6.136;
G_c=tf([K_d K_p K_i],[1 0]);
loop=feedback(series(G,G_c),1);
G_p=tf(8^5,loop.Numerator{1,1});
sys=series(G_p,loop);

impulse(sys);

%% Just Remain it Commented (But don't Delete it)

% [Num,Den] = tfdata(sys);
% syms s;
% sys_syms = poly2sym(cell2mat(Num),s)/poly2sym(cell2mat(Den),s);
% pretty(simplify(sys_syms))

%% Paper Model (Check)

% clear;
% 
% Ra=1;
% Kt=0.02;
% Kb=0.02;
% r1=0.015;
% M=1;
% m=0.1;
% l=0.25;
% g=9.81;
% 
% Kr=Kt/Ra;
% 
% c1=Kr*Kb/r1^2;
% c2=Kr/r1;
% 
% A=[0 0 1 0;
%     0 0 0 1;
%     0 -m*g/M -c1/M 0;
%     0 (M+m)*g/(M*l) c1/(M*l) 0];
% B=[0;0;c2/M;-c2/(M*l)];
% C=[1 0 0 0;
%     0 1 0 0];
% D=[0;0];
% 
% 
% [num,den]=ss2tf(A,B,C,D);
% 
% sys_Theta=tf(num(2,:),den)