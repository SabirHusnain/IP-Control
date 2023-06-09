%% Motor Parameter Estimation Code
clear; clc;
uno=arduino('COM5', 'Uno', 'BaudRate', 115200, 'Libraries', 'RotaryEncoder');

dir1='D4';
dir2='D5';
sg1='D2';
sg2='D3';

encoder=rotaryEncoder(uno,sg1,sg2,200);
writeDigitalPin(uno,dir1,0);
writeDigitalPin(uno,dir1,0);
%%
rpm=ones(100,1);
t=(0:0.1:9.9)';
dutyCycle=1*square(2*pi*0.5*t);
freq=rateControl(10);
%%
for i=1:100
    rpm(i)=readSpeed(encoder);
    if(dutyCycle(i)>0)
        temp=1;
        writeDigitalPin(uno,dir1,1);
        writeDigitalPin(uno,dir2,0);
    else
        writeDigitalPin(uno,dir1,0);
        writeDigitalPin(uno,dir2,1);
    end
    waitfor(freq);
end
%%
writeDigitalPin(uno,dir1,0);
writeDigitalPin(uno,dir2,0);

plot(t,rpm,'-xr','LineWidth',1);
hold on;
plot(t,out.simout,'-xb','LineWidth',1);
grid on;
xlabel('Time (s)');
ylabel('Speed (RPM)');
title('Motor Response');