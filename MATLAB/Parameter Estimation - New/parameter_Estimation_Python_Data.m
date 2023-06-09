clear;
clc;

load('myNewMotor4.mat');
data=readmatrix('data5.csv');
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
%%
X=w(1:end-1);S=out.simout(1:end-1);
Fs = 2839/5;            % Sampling frequency                    
T = 1/Fs;             % Sampling period       
L = 5678;             % Length of signal
t = (0:L-1)*T;

Y=fft(X);

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;
figure(2);
subplot(2,1,1);
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
%%
Y = fft(S);
P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
subplot(2,1,2);
plot(f,P1) 
title('Single-Sided Amplitude Spectrum of S(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')
%%
[filt d]=lowpass(w(1:end),10,Fs);
% filt=highpass(filt,0.5,Fs);
figure(3);
plot(t,filt,t,X,t,S)