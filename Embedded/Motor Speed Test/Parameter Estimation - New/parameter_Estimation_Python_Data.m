clear;
clc;

data=readmatrix('data.csv');
t=data(:,1)/1000000;
w=data(:,5)*(2*pi/60);

out=sim('myMotor.slx');
figure(1);
plot(t,w,'-r','LineWidth',2); hold on;
plot(out.tout,out.simout,'--b','LineWidth',2);
title('Motor Model Approximation');
xlabel('time (s)'); ylabel('Velocity (rad.s^-^1)');
grid on; grid minor;
legend('Real Motor', 'Approximated Model');