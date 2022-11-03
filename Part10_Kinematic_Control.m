
function [control_signal_1, control_signal_2, control_signal_3, error, de_new] = ...
    Kinematic_Control(theta_des_actlink1,theta_des_actlink2,theta_des_actlink3,theta1_1_feedback,theta1_2_feedback,theta1_3_feedback,error,de_new)
%% PID Controller

% kp = 8 ;  %161 for P controller  %kp for snail 164    %kp for helix 52
% J = 0.999;
% kd = 9 ;    %kd for snail 25    %kd for helix 100
% ki = 0.1 ;   %ki for snail 0.1    %ki for helix 0.1

kp = 0 ;  %161 for P controller  %kp for snail 164    %kp for helix 52
J = 1;
kd = 0 ;    %kd for snail 25    %kd for helix 100
ki = 0 ; 

theta_des = [theta_des_actlink1 theta_des_actlink2 theta_des_actlink3];
theta_feedback = [theta1_1_feedback theta1_2_feedback theta1_3_feedback];


error_old = error ;
error = (theta_des - theta_feedback).';
DE = error - error_old;
de_old=de_new;

de_new = J * de_old + DE * (1 - J);

controller_output = kp * (error) +  (ki * error + error_old) + kd * (de_new - de_old);

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
    control_signal_1 = controller_output(1);
    control_signal_2 = controller_output(2);
    control_signal_3 = controller_output(3);
    
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