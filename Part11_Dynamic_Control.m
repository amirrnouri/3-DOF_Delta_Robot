function [control_signal_1, control_signal_2, control_signal_3, error, de_new] = ...
    Dynamic_Control(theta_des_actlink1,theta_des_actlink2,theta_des_actlink3,theta1_1_feedback,theta1_2_feedback,theta1_3_feedback,error,de_new, J_x ,thetadd_des_actlink1 , thetadd_des_actlink2 , thetadd_des_actlink3)
L1 = 0.200 ;
L2 = 0.460 ;
r = 0.074 ;
R = 0.10 ;

r_cg = 1/3 ; % portion of forearm mass that is placed at the elbow
m_end_effector = 3*0.100 + 0.325 + 6*0.088 + 6*0.03 ; % mass of travelling plate
m_payload = 0 ; 
m_forearm = 2*0.150 + 4*0.053;   % mass of the forearm 
m_ee_total =  m_end_effector + m_payload + 3*(1-r_cg)*m_forearm ; % total contribution mass of traveling plate
m_upper_arm = 0.154 + 0.212 ;  % mass of the upper arm
m_middle_joint = 0.207 + 2*0.088 + 2*0.03 ;   %  mass of elbow between upper arm and forearm
I_motor = 0 ;

% m_upper_arm = 0.781;  % actuated link mass
% m_pin = 2*0.259; 
% m_par = 2*0.574; 
% m2 = m_pin + m_par;  % parallelogram link mass 
% mp = 0.637;  % end-effector mass
% Ip = zeros(6,1); % inertia of end-effector


% m_ab = m_nt ;

I_bt_1 = I_motor + (L1)^2 * ((m_upper_arm)/3 + m_middle_joint + (r_cg*m_forearm)) ; 
I_bt_2 = I_motor + (L1)^2 * ((m_upper_arm)/3 + m_middle_joint + (r_cg*m_forearm)) ; 
I_bt_3 = I_motor + (L1)^2 * ((m_upper_arm)/3 + m_middle_joint + (r_cg*m_forearm)) ; 

I_bt = [ I_bt_1 , 0 , 0 ; 0 , I_bt_2 , 0 ; 0 , 0 , I_bt_3 ] ; 

Mass_matrix = I_bt + m_ee_total * transpose(J_x) * J_x ; 


kp = 12 ;  %161 for P controller  %kp for snail 164    %kp for helix 52
J = 0.99 ;
kd = 16 ;    %kd for snail 25    %kd for helix 100
ki = 0.1 ;   %ki for snail 0.1    %ki for helix 0.1

theta_des = [theta_des_actlink1 theta_des_actlink2 theta_des_actlink3];
theta_feedback = [theta1_1_feedback theta1_2_feedback theta1_3_feedback];
thetadd_des = [thetadd_des_actlink1 thetadd_des_actlink2 thetadd_des_actlink3].' ;
error_old=error;
error = -(theta_feedback - theta_des).';
DE = error - error_old;
de_old=de_new;
de_new = J * de_old + DE * (1 - J);

controller_output = Mass_matrix*(kp * (error) + ki * (error + error_old) + kd * (de_new - de_old) + thetadd_des) ;








% r1=300;
% r2=150;
% l1=305;
% l2=608;
% th_plat=15/180*pi;
%----------------------------------------------------------
% syms alpha1 alpha2 alpha3 alpha4
% assume(alpha1,'real')
% assume(alpha2,'real')
% assume(alpha3,'real')
% assume(alpha4,'real')
 
% P_zegond_R(:,1)=[r2*cos(th_plat);r2*sin(th_plat);0];
% P_zegond_R(:,2)=[r2*sin(th_plat);r2*cos(th_plat);0];
% P_zegond_R(:,3)=[-r2*cos(th_plat);-r2*sin(th_plat);0];
% P_zegond_R(:,4)=[-r2*sin(th_plat);-r2*cos(th_plat);0];
 
% Rot_z=[cos(phi),-sin(phi),0;...
%        sin(phi), cos(phi),0;
%           0    ,    0    ,1];
       
% PR(:,1)=Rot_z*P_zegond_R(:,1)+[x;y;z];
% PR(:,2)=Rot_z*P_zegond_R(:,2)+[x;y;z];
% PR(:,3)=Rot_z*P_zegond_R(:,3)+[x;y;z];
% PR(:,4)=Rot_z*P_zegond_R(:,4)+[x;y;z];
 
% CR(:,1)=[r1+l1*sin(alpha1);0;-l1*cos(alpha1)];
% CR(:,2)=[0;r1+l1*sin(alpha2);-l1*cos(alpha2)];
% CR(:,3)=[-r1-l1*sin(alpha3);0;-l1*cos(alpha3)];
% CR(:,4)=[0;-r1-l1*sin(alpha4);-l1*cos(alpha4)];
 
% for i=1:4
% eq(i)=(CR(1,i)-PR(1,i))^2+(CR(2,i)-PR(2,i))^2+(CR(3,i)-PR(3,i))^2-l2^2;
% end
 
% Alpha1=eval(vpasolve(eq(1),alpha1))
% Alpha1=Alpha1*180/pi;
% Alpha2=eval(vpasolve(eq(2),alpha2))
% Alpha2=Alpha2*180/pi;
% Alpha3=eval(vpasolve(eq(3),alpha3))
% Alpha3=Alpha3*180/pi;
% Alpha4=eval(vpasolve(eq(4),alpha4))
% Alpha4=Alpha4*180/pi;
%------------------------------------------------------------
% Alpha1 = -0.5695
% Alpha2 = -0.5018
% Alpha3 =  0.3654
% Alpha4 =  0.6538
% -----------------------------------------------------------
% theta = [theta1_1_feedback theta1_2 theta1_3_feedback];
%de_old = 0;
% if(norm(error) < 0.001)
%     u1 = theta(1);
%     u2 = theta(2);
%     u3 = theta(3);
% else
%.........................sliding_2.............................
% pos = [R1 R2 R3 R4];
% J = 0.95;
% delta_star = 5;
% error = pos - theta;
% error_old = error;
% DE = error - error_old;
% de_new = J * de_old + (DE) * (1 - J);
% vel = (de_new) - delta_star * tanh(100*error);
% de_old = de_new;
% %..........................Sliding_Mode..........................
% % speed = [0.05 0.05 0.05 0.05];
% k1 = 1;
% k2 = 0.5;
% J = 0.95;
% delta_star = 5;
% %velocity = [V1 V2 V3 V4];
% pos = [R1 R2 R3 R4];
% % x1 = theta - pos;
% % x2 = speed - velocity; % x2 = de_new - de_old
% error = pos - theta;
% error_old = error;
% DE = error - error_old;
% de_new = J * de_old + (DE) * (1 - J);
% S = k1*(error) + k2*(de_new);
% vel = (-k1/k2)*(de_new) - delta_star * tanh(50*S) ; %sign(S)
% de_old = de_new;



%% PID Controller

% kp = 8 ;  %161 for P controller  %kp for snail 164    %kp for helix 52
% J = 0.999;
% kd = 5 ;    %kd for snail 25    %kd for helix 100
% ki = 0.5 ;   %ki for snail 0.1    %ki for helix 0.1
% 
% theta_des = [theta_des_actlink1 theta_des_actlink2 theta_des_actlink3];
% theta_feedback = [theta1_1_feedback theta1_2_feedback theta1_3_feedback];
% 
% error_old=error;
% error = -(theta_feedback - theta_des).';
% DE = error - error_old;
% de_old=de_new;
% de_new = J * de_old + DE * (1 - J);
% 
% controller_output = kp * (error) + ki * (error + error_old) + kd * (de_new - de_old);

%..................PD_controller_with_filter_low_pass.........................

% error = error + rand *1e-12;
% de_new = de_new + rand *1e-10;

%..................PD_controller_with_filter_low_pass.........................

% kp = 45;
% J = 0.999;
% pos = [R1 R2 R3 R4];
% kd = 4.5;
% error_old = error
% error = theta - pos;
% DE = error - error_old;
% de_new = J * de_old + (DE) * (1 - J);
% vel = kp * (error) + kd * (de_new - de_old);
% de_old = de_new;

%.............................PD_Controller_without_filter_low_pass...............................    

% kp = 45;  % vaghti kp ro kam kardam deghat tracking paeen oumad.
% pos = [R1 R2 R3 R4];
% kd = 0.5;
% error_old = error
% error = theta - pos;
% % error1 = [error1 ; error]; 
% vel = kp * (error) + kd * (error - error_old);  % moshkel:hamishe yek offset 
% % beyn desire o real hasteh ke ba yek filter paeen gozar dorost mishe.

% --------------------------P_controller------------------------

% kp = 45;  % vaghti kp ro kam kardam deghat tracking paeen oumad.
% pos = [R1 R2 R3 R4];
% error = theta - pos;
% % error1 = [error1 ; error];
% vel = kp * (error);
% % for i=1:4
% %     if(vel(i) > 5)
% %         vel(i) = 5;
% %     end
% % end

%------------------------------------------------------------

% for Position input
%     u1 = vel(1) + R1
%     u2 = vel(2) + R2
%     u3 = vel(3) + R3
%     u4 = vel(4) + R4

%------------------------------------------------------------

% for velocity input
    control_signal_1 = controller_output(1)/1000;
    control_signal_2 = controller_output(2)/1000;
    control_signal_3 = controller_output(3)/1000;
    
%------------------------------------------------------------
% for i=1:4
%     u1 = vel(1)=0;
%     u2 = vel(2)=0;
%     u3 = vel(3)=0;
%     u4 = vel(4)=0;
% end
% y = u;
% end
end
