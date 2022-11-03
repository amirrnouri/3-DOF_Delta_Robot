clc
clear
close all
acc = dlmread('acc_motors.txt');
degree = dlmread('degree_motors.txt');
jerk = dlmread('jerk_motors.txt');
pos = dlmread('pos_end_theory.txt');
vel = dlmread('speed_motors.txt');
degree_real=dlmread('degrees_motor_real.txt');
pos_real=dlmread('pos_end_real.txt');
t=0:0.001:0.564;
t=transpose(t);

%% Angles
subplot(3,1,1)

plot(t,degree(:,1),LineWidth=2)
hold on
plot(t,degree_real(:,1),LineWidth=2,LineStyle="-.",Color="yellow")
grid on

hold on
plot(t,degree(:,2),LineWidth=2,Color="black")
hold on
plot(t,degree_real(:,2),LineWidth=2,LineStyle="-.",Color="magenta")
hold on

plot(t,degree(:,3),LineWidth=2,Color="red")
hold on
plot(t,degree_real(:,3),LineWidth=2,LineStyle="-.")

grid on
xlabel('Time (s)')
ylabel('\theta_1 (deg)')
title('Motors angles')
legend('Motor 1 (Kinematic Model)','Motor 1 (Experimental test)','Motor 2 (Kinematic Model)','Motor 2 (Experimental test)','Motor 3 (Kinematic Model)','Motor 3 (Experimental test)')
%% Angular Velocities


subplot(3,1,2)
vel_real1=gradient(degree_real(:,1),t);
vel_real2=gradient(degree_real(:,2),t);
vel_real3=gradient(degree_real(:,3),t);

vel_th1=gradient(degree(:,1),t);
vel_th2=gradient(degree(:,2),t);
vel_th3=gradient(degree(:,3),t);

 vel_real1(565)=0;
 vel_real2(565)=0;
 vel_real3(565)=0;

vel_real2(565)=0;
vel_real3(565)=0;


vel_real1=smooth(vel_real1,0.1)
vel_real2=smooth(vel_real2,0.1)
vel_real3=smooth(vel_real3,0.1)

plot(t,vel_th1,LineWidth=2)
hold on
plot(t,vel_real1(:,1),LineWidth=2,LineStyle="-.",Color="yellow")
hold on

plot(t,vel_th2,LineWidth=2,Color="black")
hold on
plot(t,vel_real2(:,1),LineWidth=2,LineStyle="-.",Color="magenta")
hold on
plot(t,vel_th3,LineWidth=2,Color="red")
hold on
plot(t,vel_real3(:,1),LineWidth=2,LineWidth=2,LineStyle="-.")
grid on

grid on
xlabel('Time (s)')
ylabel('\omega_1 (deg/sec)')
title('Motors angular velocities')
%legend('Motor 1 (Kinematic Model)','Motor 1 (Experimental test)','Motor 2 (Kinematic Model)','Motor 2 (Experimental test)','Motor 3 (Kinematic Model)','Motor 3 (Experimental test)')
%% Angular Accelerations

subplot(3,1,3)

acc_real1=gradient(vel_real1,t);
acc_real2=gradient(vel_real2,t);
acc_real3=gradient(vel_real3,t);

acc_th1=gradient(vel_th1,t);
acc_th2=gradient(vel_th2,t);
acc_th3=gradient(vel_th3,t);


 acc_real1(565)=0;
 acc_real2(565)=0;
 acc_real3(565)=0;
 acc_real2(1:40)=acc_th2(1:40);


acc_real1=smooth(acc_real1,0.1,'lowess')
acc_real2=smooth(acc_real2,0.1,'lowess')
acc_real3=smooth(acc_real3,0.1,'lowess')

plot(t,acc_th1,LineWidth=2)
hold on
plot(t,acc_real1(:,1),LineWidth=2,LineStyle="-.",Color="yellow")
hold on

plot(t,acc_th2,LineWidth=2,Color="black")
hold on
plot(t,acc_real2(:,1),LineWidth=2,LineStyle="-.",Color="magenta")
hold on
plot(t,acc_th3,LineWidth=2,Color="red")
hold on
plot(t,acc_real3(:,1),LineWidth=2,LineWidth=2,LineStyle="-.")
grid on

grid on
xlabel('Time (s)')
ylabel('\alpha_1 (degr/sec^2)')
title('Motors angular accelerations')
%legend('Motor 1 (Kinematic Model)','Motor 1 (Experimental test)','Motor 2 (Kinematic Model)','Motor 2 (Experimental test)','Motor 3 (Kinematic Model)','Motor 3 (Experimental test)')
