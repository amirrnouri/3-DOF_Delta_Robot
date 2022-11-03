
clc
clear
M = dlmread('position1.txt')

zlim([-.600 -0.2])

r=0.15;
i=1
for t=0:0.01:2*pi
x(i)=r*cos(t);
y(i)=r*sin(t);
z(i)=-0.4;
i=i+1;
end

load('circle_zlist.mat')
load('circle_ylist.mat')
load('circle_xlist.mat')

% desired path:
plot3(circle_xlist,circle_ylist,circle_zlist,LineWidth=1)
hold on

% simulation results:
plot3(X_EndEffector,Y_EndEffector,Z_EndEffector,LineStyle="--",LineWidth=2)

% Experimantal test:
plot3(M(:,1),M(:,2),M(:,3),LineStyle="-.",LineWidth=2)

xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
legend('Kinematic Model','Simscape Model','Experimental Test')


grid on
%%
load('circle_t2list.mat')
load('circle_t3list.mat')
load('circle_t1list.mat')
T = dlmread('motors.txt')*pi/180;
b=(1:4820)/4820*6.29;

t1=circle_t1list;
t2=circle_t2list;
t3=circle_t3list;
a=(1:629)/100;
c=(1:4947)/4947*6.29;


simt1=Theta_ActJoint_1;
simt2=Theta_ActJoint_2;
simt3=Theta_ActJoint_3;

subplot(3,1,1)
plot(a,t1,LineWidth=2)
hold on
plot(a,t1,LineWidth=2,LineStyle="--")

plot(b,T(:,2),LineWidth=2,LineStyle="-.")

grid on
xlabel('Time (s)')
ylabel('\theta_1 (rad)')
title('Motor1')
legend('Kinematic Model','Simscape Model','Experimental Test')



subplot(3,1,2)
plot(a,t2,LineWidth=2)

hold on
plot(a,t2,LineWidth=2,LineStyle="--")

hold on
plot(b,T(:,3),LineWidth=2,LineStyle="-.")

xlabel('Time (s)')
ylabel('\theta_1 (rad)')
grid on
title('Motor2')
legend('Kinematic Model','Simscape Model','Experimental Test')


subplot(3,1,3)
plot(a,t3,LineWidth=2)
hold on
plot(a,t3,LineWidth=2,LineStyle="--")
hold on
plot(b,T(:,1),LineWidth=2,LineStyle="-.")
grid on
xlabel('Time (s)')
ylabel('\theta_1 (rad)')
title('Motor3')
legend('Kinematic Model','Simscape Model','Experimental Test')


