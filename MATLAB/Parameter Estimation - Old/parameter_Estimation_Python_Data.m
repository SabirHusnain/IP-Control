clear;
clc;

load('myNewMotor.mat');
data=readmatrix('data3.csv');
t=data(:,1)/1000000;
w=data(:,5)*(2*pi/60);
dir=data(:,4);

out=sim('myMotor.slx');
figure(1);
plot(t,w,out.tout,out.simout);

figure(2);
plot(t,dir);