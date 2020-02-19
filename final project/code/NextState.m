% Milestone 1: youBot Kinematics Simulator and csv Output
% NextState

%function Trobot_nextstate ...
%         = NextState(Trobot_currentstate, control_input, timestep)

function Trobot_nextstate ...
         = NextState(Trobot_currentstate, control_input, timestep, limits)

%% define input and output
% INPUT                                                

% Trobot_currentstate             
% A 12-vector representing the current configuration of the robot 
% (3 variables for the chassis configuration, 5 variables for the arm 
% configuration, and 4 variables for the wheel angles).

% control_input
% A 9-vector of controls indicating the arm joint speeds (5 variables) 
% and the wheel speeds u (4 variables).

% timestep
% A timestep delta t

% limits
% A positive real value indicating the maximum angular speed of the arm 
% joints and the wheels.


% OUTPUT

% Trobot_nextstate
% A 12-vector representing the configuration of the robot after delta t

%% update arm joints angles, wheel speeds

% arm joints
arm_joints_angles_k = zeros(5,1);
% arm_joints_angles_kplus1 = zeros(5,1);
joint_speeds = zeros(5,1);
joint_speeds_limits = zeros(5,1);
for i = 1: 5
    joint_speeds_limits(i,1) = limits(i,1);
    arm_joints_angles_k(i,1) = Trobot_currentstate(i+3,1);  
    joint_speeds(i,1) = control_input(i,1);
    % arm joint speeds limit check
    if joint_speeds(i,1) > joint_speeds_limits(i,1)
        joint_speeds(i,1) = joint_speeds_limits(i,1);
    elseif joint_speeds(i,1) < -joint_speeds_limits(i,1)
        joint_speeds(i,1) = -joint_speeds_limits(i,1);
    else
        joint_speeds(i,1) = joint_speeds(i,1); 
    end
end

% wheel angles
wheel_angles_k = zeros(4,1);
% wheel_angles_kplus1 = zeros(4,1);
wheel_speeds = zeros(4,1);
wheel_speeds_limits = zeros(4,1);
 for i = 1: 4
    wheel_speeds_limits(i,1) = limits(i+5,1);
    wheel_angles_k(i,1) = Trobot_currentstate(i+8,1);
    wheel_speeds(i,1) = control_input(i+5,1);
    % wheel angles speeds limit check
    if wheel_speeds(i,1) > wheel_speeds_limits(i,1)
        wheel_speeds(i,1) = wheel_speeds_limits(i,1);
    elseif wheel_speeds(i,1) < -wheel_speeds_limits(i,1)
        wheel_speeds(i,1) = -wheel_speeds_limits(i,1);
    else
        wheel_speeds(i,1) = wheel_speeds(i,1);
    end
 end
 
% update to time k+1
 arm_joints_angles_kplus1 = arm_joints_angles_k + joint_speeds*timestep;
 wheel_angles_kplus1 = wheel_angles_k + wheel_speeds*timestep; 
 
 
%% finally we update the chassis configuration
 
chassis_configuration_k = zeros(3,1); 
% chassis_configuration_kplus1 = zeros(3,1);
 
 for i = 1: 3
    chassis_configuration_k(i,1) = Trobot_currentstate(i,1);  
 end 
 
% The forward-backward distance between the wheels is 2l = 0.47 meters and 
%the side-to-side distance between wheels is 2w = 0.3 meters. 
% The radius of each wheel is r = 0.0475 meters. 

l = 0.47/2;
w = 0.3/2;
r = 0.0475;
b = l+w;

% Vb = zeros(3,1);
Vb = (r/4)*[-1/b 1/b 1/b -1/b; 1 1 1 1; -1 1 -1 1]*wheel_speeds*timestep;
% Vb = F * delta theta, eq13.33 for four-mecanum-wheel robot
wbz = Vb(1,1);
vbx = Vb(2,1);
vby = Vb(3,1);

%{
Vb6 = zeros(6,1);
Vb6 = [0;0;Vb;0];
Vb6_w = zeros(3,1);
Vb6_v = zeros(3,1);
 for i = 1: 3
    Vb6_w(i,1) = Vb6(i,1)  
 end 

 for i = 1: 3
    Vb6_v(i,1) = Vb6(i+3,1)  
 end 
w_so3 = zeros(3,3); 
w_so3 = VecTose3(Vb6_w);
%Returns the skew symmetric matrix in so(3).

V = zeros(4,4);
V = [w_so3 Vb6_v; 0 0 0 0]; %se(3)

T = zeros(4,4);
T = MatrixEx6(V); %T_bk_bk+1, Tbb'
%}

delta_qb = zeros(3,1);
    if wbz == 0
        delta_qb = [0; vbx; vby];
    else
        delta_qb = [wbz; (vbx*sin(wbz)+vby*(cos(wbz)-1))/wbz; (vby*sin(wbz)+vbx*(1-cos(wbz)))/wbz];
    end

phi_k = chassis_configuration_k(1,1);
delta_q = zeros(3,1);
delta_q = [1 0 0; 0 cos(phi_k) -sin(phi_k); 0 sin(phi_k) cos(phi_k)]*delta_qb;

chassis_configuration_kplus1 = chassis_configuration_k + delta_q;

Trobot_nextstate = [chassis_configuration_kplus1; arm_joints_angles_kplus1; wheel_angles_kplus1];

end