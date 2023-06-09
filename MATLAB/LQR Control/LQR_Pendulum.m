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

Q=[50 0 0 0;
    0 1 0 0;
    0 0 50 0;
    0 0 0 1];
R=0.30;
[K,P,E]=lqr(A,B,Q,R);

%% Controller System Bahvaiour
t=0:0.01:5;
sys=ss(A-B*K,[],eye(4),[]);
x_Loop=initial(sys, [0.2;0;-0.20;0], t);

figure(1);
subplot(3,2,1);
plot(t,x_Loop(:,1),'-r', 'LineWidth', 2); grid on; grid minor;
title('Close Loop System State x_1');
xlabel('time (s)'); ylabel('Amplitude');

subplot(3,2,2);
plot(t,x_Loop(:,2),'-g', 'LineWidth', 2); grid on; grid minor;
title('Close Loop System State x_2');
xlabel('time (s)'); ylabel('Amplitude');

subplot(3,2,3);
plot(t,x_Loop(:,3),'-b', 'LineWidth', 2); grid on; grid minor;
title('Close Loop System State x_3');
xlabel('time (s)'); ylabel('Amplitude');

subplot(3,2,4);
plot(t,x_Loop(:,4),'-m', 'LineWidth', 2); grid on; grid minor;
title('Close Loop System State x_4');
xlabel('time (s)'); ylabel('Amplitude');

U=-K*x_Loop';
subplot(3,2,[5 6]);
plot(t,U,'-k', 'LineWidth', 2); grid on; grid minor;
title('Control Signal');
xlabel('time (s)'); ylabel('Amplitude');