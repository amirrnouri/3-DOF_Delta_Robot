ii=1;
for x=-0.7:0.01:0.7
    for y=-0.7:0.01:0.7
        for z=-0.01:-0.01:-0.7

            
            [theta_des_actlink1,theta_des_actlink2,theta_des_actlink3,...
          thetad_des_actlink1,thetad_des_actlink2,thetad_des_actlink3,...
          thetadd_des_actlink1,thetadd_des_actlink2,thetadd_des_actlink3, J_x]=Inverse_Kinematics(x, y, z, 0,0, 0, 0, 0, 0);
            abs(det(J_x))
            if abs(det(J_x))<10^-4
            singularx(ii)=x
            singulary(ii)=y
            singularz(ii)=z
            ii=ii+1
        end
        end
    end
end


function [theta_des_actlink1,theta_des_actlink2,theta_des_actlink3,...
          thetad_des_actlink1,thetad_des_actlink2,thetad_des_actlink3,...
          thetadd_des_actlink1,thetadd_des_actlink2,thetadd_des_actlink3, J_x] = ...
      Inverse_Kinematics(x_des, y_des, z_des, xd_des, yd_des, zd_des, xdd_des, ydd_des, zdd_des)
  
      
  % these lines are executed because of coordinate system rotation:
  y_des=-y_des;
    z_des=-z_des;


L1 = 0.20 ;
L2 = 0.46 ;
r = 0.074 ;
R = 0.1;
% t = 0:0.001:10 ;

%% Inverse Kinematics


psi = zeros(3,1);
psi(1,1) =  0;
psi(2,1) = 2*pi/3;
psi(3,1) = -2*pi/3;

P_ee = zeros(3,3) ;
V_ee = zeros(3,3) ;
A_ee = zeros(3,3) ;

% k = length(t) ;
q = zeros(3,3) ;    
Rot_0_to_global = zeros(3,9) ;
ti = zeros(3,1) ;
error = zeros(3,1) ;

phi = zeros(3,1) ;
theta2 = zeros(3,1) ;
theta1 = zeros(3,1) ;

phi_dot = zeros(3,1);
theta1_dot = zeros(3,1);
theta2_dot = zeros(3,1);

theta1_ddot = zeros(3,1);


J_theta1 = zeros(3,3) ; 

x_ee = x_des ;
y_ee = y_des ;
z_ee = z_des ;
    
xd_ee = xd_des ;
yd_ee = -yd_des ;
zd_ee = -zd_des ;
    
xdd_ee = xdd_des ;
ydd_ee = -ydd_des ;
zdd_ee = -zdd_des ;

        for i=1:3
        
            Rot_02g = [ cos(psi(i,1))   sin((psi(i,1)))   0 ;...
                        -sin(psi(i,1))   cos(psi(i,1))    0 ;...
                            0                 0           1 ]; % 3x3
                        
            Rot_0_to_global(:,-2 + 3*i:3*i) = Rot_02g ; 

            P_ee(:,i) = Rot_0_to_global(:,-2 + 3*i:3*i) * [x_ee ; y_ee ; z_ee] ;  % 3x1
            V_ee(:,i) = Rot_0_to_global(:,-2 + 3*i:3*i) * [xd_ee ; yd_ee ; zd_ee] ; % 3x1
            A_ee(:,i) = Rot_0_to_global(:,-2 + 3*i:3*i) * [xdd_ee ; ydd_ee ; zdd_ee] ; % 3x1

           
        
            q(:,i) = P_ee(:,i) + [r-R ; 0 ;0]; % 3x1
            qx = q(1,i); % 1x1
            qy = q(2,i); % 1x1
            qz = q(3,i); % 1x1
        
            phi(i,1) = acos(qy/L2);
        
            a = (qx+L1)^2 + qz^2 - (L2*sin(phi(i,1)))^2 ; 
            b = -4*L1*qz;  % b is defined as b prime in delta equation
            c=(qx-L1)^2+qz^2-(L2*sin(phi(i,1)))^2; % 1x1
        
                if ((b^2)>(4*(a*c)))
                    error(i,1) = 0 ;
                    ti(i,1) =(-b-(sqrt(b^2-(4*(a*c)))))/(2*a); % 1x1
                else
                    error(i,1) = 1 ;
                    ti(i,1) = 'e' ;
                end
            
            % Equation 7- The planar angle between upper and lower link
            theta2(i,1) = acos((qx^2 + qy^2 + qz^2 - L2^2 - L1^2)/(2*L1*L2*sin(phi(i,1)))); % 1x1
            % Equation 14- Actuated Link Angle (theta1)
            theta1(i,1) = 2*atan(ti(i,1)); % 1x1  
            
            % in the arm coordinate system
            L1_vector = L1*[cos(theta1(i,1)) ; 0 ; sin(theta1(i,1))]; % 3x1
            L2_vector = L2*[sin(phi(i,1))*cos(theta1(i,1)+theta2(i,1)) ; ...
            cos(phi(i,1)) ; sin(phi(i,1))*sin(theta1(i,1)+theta2(i,1))]; % 3x1
        
            
            s1 = [0;-1;0]; % 3x1
            s2 = [-sin(theta1(i,1)+theta2(i,1)) ; 0 ; cos(theta1(i,1)+theta2(i,1))]; % 3x1

            
            cross_l2l1 = cross(L2_vector,L1_vector); % 3x1
            cross_s2s1 = cross(s2,s1); % 3x1
        
            J_theta1(i,:) = -(1*L2_vector.')/dot(cross_l2l1,s1); % 1x3

            phi_dot(i,1) = -(s1.' * V_ee(:,i))/(dot(cross_s2s1,L2_vector)); % 1x1
            theta1_dot(i,1) = -(L2_vector.' * V_ee(:,i))/(dot(cross_l2l1,s1)); % 1x1
            theta2_dot(i,1) = -( (L1_vector.' * V_ee(:,i) + (1)*((dot(cross_l2l1,s2))/(dot(cross_s2s1,L2_vector)))* s1.' * V_ee(:,i))/(-dot(cross_l2l1,s1)) ); % 1x1
   
           
            
            theta1_ddot(i,1) = -( dot((A_ee(:,i)+ L1_vector*theta1_dot(i,1)^2),L2_vector) + (2*theta2_dot(i,1)*phi_dot(i,1)*dot(s1,s2)*cos(phi(i,1))*L2^2) + (((sin(phi(i,1)))^2*theta2_dot(i,1)^2+phi_dot(i,1)^2)*L2^2 ))/(dot(cross_l2l1,s1)); % 1x1
    
          
           
            
        end
    
theta_des_actlink1 = theta1(1,1) ;
theta_des_actlink2 = theta1(2,1) ;
theta_des_actlink3 = theta1(3,1) ;

thetad_des_actlink1 = theta1_dot(1,1) ;
thetad_des_actlink2 = theta1_dot(2,1) ;
thetad_des_actlink3 = theta1_dot(3,1) ;

thetadd_des_actlink1 = theta1_ddot(1,1) ; 
thetadd_des_actlink2 = theta1_ddot(2,1) ; 
thetadd_des_actlink3 = theta1_ddot(3,1) ; 

J_x = inv(J_theta1) ; 


end

