% please add all the written functions to the mr folder and include the mr foler 
% for all the configurations written below, we use T for robot and X for end-effector
addpath(genpath('mr')) %include Modern Robotics library

%% Important reminder 
% caution, when testing different controller and cube's configurations,
% please make sure that the following variables have been changed.
% 'Xsc_initial' for cube's initial configuration
% 'Xsc_initial' for cube's final configuration
% 'a' and 'Num_Configurations' will decide how many configurations that will
% be generated, for initial setting, please use (a, Num_Configurations)=
% (2,2000), for the newTask, plase use (4,4000)
% A total six set of 'Kp' and 'Ki' gain are set for feedforward only,
% feedforward plus P, and feedforward plus PI

%% step 1, generate reference trajectory

% give a 30 degrees orientation error and 0.2m of position error
% Rx_30 = [1 0 0; 0 cos(pi/6) -sin(pi/6); 0 sin(pi/6) cos(pi/6)];
% Terror = [Rx_30 [0.2; 0; 0]; 0 0 0 1];
Xse_initial = [0 0 1 0; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
% Xse_initial = [0, 0, 1, 0; 0, 1, 0, 0; -1, 0, 0, 0.5; 0, 0, 0, 1]; % function test
% Xse_initial = [1, 0, 0, -1; 0, 1, 0, 0; 0, 0, 1, 0.4012; 0, 0, 0, 1]; % function test
% initial configuratino of the end effector in the reference trajectory

% Xsc_initial = [1, 0, 0, 2; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1]; % newTask
Xsc_initial = [1, 0, 0, 1; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1]; % initial value of scene 6
%the cube's initial configuration

% Xsc_fianl = [0, 1, 0, 0; -1, 0, 0, -2; 0, 0, 1, 0; 0, 0, 0, 1]; % newTask
Xsc_fianl = [0, 1, 0, 0; -1, 0, 0, -1; 0, 0, 1, 0; 0, 0, 0, 1]; % initial value of scene 6
%the cube's final configuration

Xce_grasp = [0, 0, 1, 0; 0, 1, 0, 0; -1, 0, 0, 0.025; 0, 0, 0, 1]; 
%configuration of the end effector {e} relative to the cube frame {c} when
%the cube is held by the gripper

Xce_standoff = [0, 0, 1, 0; 0, 1, 0, 0; -1, 0, 0, 0.225; 0, 0, 0, 1]; % function test
%end effector's standoff configuration above the cube, before and after
%grasping, relative to the cube

k =1;
%The number of reference configurations per 0.01 seconds: k

a = 2; % initial value of scene 6
% a = 4; % newTask
% Num_Configurations = a*1000

X_reference_traj = TrajectoryGenerator(Xse_initial, Xsc_initial, Xsc_fianl, ...
                                     Xce_grasp, Xce_standoff, k, a);
                                 
% with k = 1, we generate Num_Configurations reference configurations
% the total operation time has been set as Num_Configurations/100 sec 

%% step 2, recursive algorithm 

% recall that Num_Configurations reference configurations were generated, hence there
% shall be Num_Configurations-1 loops

% initilization
Num_Configurations = 2000; % newTask % initial value of scene 6
% Num_Configurations = 4000; % newTask
log_Xerr = zeros(Num_Configurations,6); % for csv and plotting
log_robot_configuration = cell(Num_Configurations,1); % for csv and plotting


% for initial cube configurations in scene 6, please use this set
% Kp = 0*eye(6,6); % before tuning 
% Ki = 0*eye(6,6); % before tuning

% Kp = 2*eye(6,6); % test, overshoot
% Ki = 1.2*eye(6,6); % test, overshoot

Kp = 2*eye(6,6) % test, best
Ki = 0*eye(6,6) % test, best

% for the newTask cube configurations in scene 6, please use this set, and
% be sure to tune the a variable in TrajectoryGenerator function
% Kp = 0*eye(6,6); % before tuning 
% Ki = 0*eye(6,6); % before tuning

% Kp = 3*eye(6,6); % test, overshoot
% Ki = 1*eye(6,6); % test, overshoot

% Kp = 3*eye(6,6); % test, best
% Ki = 0*eye(6,6); % test, best

% limits for the maximum angular speed of the arm joints and the wheels
limits = 10*ones(9,1); % the maximum angular speed 
timestep = 0.01;
% T = [0 0 0 0 0 0.2 -1.6 0]; % initial guess
T = zeros(1,12); % initial guess 
log_robot_configuration{1,1} = [T X_reference_traj(1,13)];
% A 12-vector representing the current configuration of the robot 
% (3 variables for the chassis configuration, 5 variables for the arm 
% configuration, and 4 variables for the wheel angles).
Xerr_integral = zeros(6,1); % the integral of the error

for loop = 1:Num_Configurations-1
temp_X0 = zeros(1,12); % to store the configuration but w/o gripper state
temp_Xd = zeros(1,12); % to store the configuration but w/o gripper state
temp_Xd_next = zeros(1,12); % to store the configuration but w/o gripper state
for j = 1:12
    temp_X0(1,j) = X_reference_traj(1,j); % for comparing the initial guess
    temp_Xd(1,j) = X_reference_traj(loop,j);
    temp_Xd_next(1,j) = X_reference_traj(loop+1,j);
end
Xd = temp_Xd; % dimension 1*12
Xd_next = temp_Xd_next; %dimension 1*12
% the abive configurations have a dimension of 1*12, however, we shall turn
% them into 4*4 to fit the input format in FeedbackControl function
temp_X0_configuration = [temp_X0(1:3) temp_X0(10); temp_X0(4:6) temp_X0(11); temp_X0(7:9) temp_X0(12); 0 0 0 1]; 
Xd_configuration = [Xd(1:3) Xd(10); Xd(4:6) Xd(11); Xd(7:9) Xd(12); 0 0 0 1]; 
Xd_next_configuration = [Xd_next(1:3) Xd_next(10); Xd_next(4:6) Xd_next(11); Xd_next(7:9) Xd_next(12); 0 0 0 1]; 

%% X update
% Jarm
theta = zeros(5,1); % initialization of the arm joints value
for j = 1:5
    theta(j,1) = T(j+3);
end
Blist = [[0; 0; 1;   0; 0.033; 0], ...
         [0; -1; 0;   -0.5076;   0;   0], ...
         [0; -1; 0;   -0.3526;   0;   0], ...
         [0; -1; 0;   -0.2176;   0;   0], ...
         [0; 0; 1; 0; 0; 0]];
thetalist = theta;
Jarm = JacobianBody(Blist, thetalist);
% Jbase
phi = T(1);
x = T(2);
y = T(3);
Tsb_q = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];
Tb0 = [eye(3) [0.1662; 0; 0.0026]; 0 0 0 1];
M0e = [eye(3) [0.033; 0; 0.6546]; 0 0 0 1];
T0e = FKinBody(M0e, Blist, thetalist);

X = Tsb_q*Tb0*T0e; % X update
X_configuration = X;

% compare the initial guess and desire end-effector configuration
if loop ==1
text = 'compare the initial guess with the Xd,1';
disp(text);
initial_guess = X_configuration;
text = 'initial_guess value of X1';
disp(text);
value(initial_guess)
Xd_1 = temp_X0_configuration;
text = 'Xd,1 value';
disp(text);
value(Xd_1)
else
end

%% U update
l = 0.47/2;
w = 0.3/2;
r = 0.0475;
b = l+w;
F = (r/4)*[-1/b 1/b 1/b -1/b; 1 1 1 1; -1 1 -1 1];
F6 = [zeros(1,4); zeros(1,4); F; zeros(1,4)];
Jbase = Adjoint(TransInv(T0e)*TransInv(Tb0))*F6;
Je = [Jbase Jarm];

% feedback control to generate a twist for wheels and arm joints
Twistee_Integralofeorror ...
         = FeedbackControl(X_configuration, Xd_configuration, Xd_next_configuration, Kp, Ki, timestep, Xerr_integral);
% output return Twist, Xerr, and the integral of the error
command_twist = Twistee_Integralofeorror(:,1);
log_Xerr(loop, :) = Twistee_Integralofeorror(:,2)'; % store Xerr of every configuration
Xerr_integral = Twistee_Integralofeorror(:,3);
% Xerr_integral{loop,1} = Xerr_integral;
% turn the end-effector twist into wheel and arm joints speed
wheel_arm_joint_speeds = pinv(Je)*command_twist; % U update


%% T update
Trobot_currentstate = T'; % T has a dimension of 1*12
% Trobot_currentstate = [0; 0; 0; 1; 0; 0; 0; 0; 0; 0; 0; 0]; % function test
% A 12-vector representing the current configuration of the robot 
% (3 variables for the chassis configuration, 5 variables for the arm 
% configuration, and 4 variables for the wheel angles).

% A 9-vector of controls indicating the arm joint speeds (5 variables) 
% and the wheel speeds u (4 variables).
% control_input = [1; 0; 0; 0; 0; 10; 10; 10; 10]; % function test
control_input = [wheel_arm_joint_speeds(5:9,1); wheel_arm_joint_speeds(1:4,1)];
% limits = zeros(9,1); 
% limits for the maximum angular speed of the arm joints and the wheels
% limits = 100*ones(9,1); % the maximum angular speed 

% a array for storing the next configuration of the robot
gripper_open = 0;
Trobot = zeros(1,13); 
% all the configuration during the operation, including gripper state

Trobot_nextstate = NextState(Trobot_currentstate, control_input, timestep, limits);
% Trobot_currentstate = Trobot_nextstate;
    for a = 1: 12
    Trobot(1,a) = Trobot_nextstate(a,1);
    end
    Trobot(1,13) = gripper_open; % this gripper state will not be updated 
    % the gripper state is directly updated in the log_robot_configuration
    % which will later be written into a csv file.
    
T = Trobot(:,1:12); %T update
log_robot_configuration{loop+1,1} = [T X_reference_traj(loop+1,13)]; % store T of every configuration

 end

%  X update for the last loop
if loop==Num_Configurations-1 % T is in the Num_Configurations th state
temp_Xd = zeros(1,12); % to store the configuration but w/o gripper state
temp_Xd_next = zeros(1,12); % to store the configuration but w/o gripper state
for j = 1:12
    temp_Xd(1,j) = X_reference_traj(loop+1,j);
    temp_Xd_next(1,j) = X_reference_traj(loop+1,j);
end
Xd = temp_Xd; % dimension 1*12
Xd_next = temp_Xd_next; %dimension 1*12
% the abive configurations have a dimension of 1*12, however, we shall turn
% them into 4*4 to fit the input format in FeedbackControl function
Xd_configuration = [Xd(1:3) Xd(10); Xd(4:6) Xd(11); Xd(7:9) Xd(12); 0 0 0 1]; 
Xd_next_configuration = [Xd_next(1:3) Xd_next(10); Xd_next(4:6) Xd_next(11); Xd_next(7:9) Xd_next(12); 0 0 0 1]; 

% Jarm
theta = zeros(5,1); % initialization of the arm joints value
for j = 1:5
    theta(j,1) = T(j+3);
end
Blist = [[0; 0; 1;   0; 0.033; 0], ...
         [0; -1; 0;   -0.5076;   0;   0], ...
         [0; -1; 0;   -0.3526;   0;   0], ...
         [0; -1; 0;   -0.2176;   0;   0], ...
         [0; 0; 1; 0; 0; 0]];
thetalist = theta;
Jarm = JacobianBody(Blist, thetalist);
% Jbase
phi = T(1);
x = T(2);
y = T(3);
Tsb_q = [cos(phi) -sin(phi) 0 x; sin(phi) cos(phi) 0 y; 0 0 1 0.0963; 0 0 0 1];
Tb0 = [eye(3) [0.1662; 0; 0.0026]; 0 0 0 1];
M0e = [eye(3) [0.033; 0; 0.6546]; 0 0 0 1];
T0e = FKinBody(M0e, Blist, thetalist);

X = Tsb_q*Tb0*T0e; % X update
X_configuration = X;
Xerr_se3 = MatrixLog6(TransInv(X)*Xd_configuration);
Xerr = se3ToVec(Xerr_se3);
log_Xerr(Num_Configurations, :) = Xerr'; % store the last Xerr
else
end

%% csv and plot
text = 'Show the Xerr of the first and final configuration';
disp(text);
text = 'initial Xerr';
disp(text);
value(log_Xerr(1,:))

text = 'last Xerr';
disp(text);
value(Xerr')

csvwrite('log_Reference_Trajectoy.csv',X_reference_traj);
csvwrite('log_T_Robot_Configuration.csv',log_robot_configuration);
csvwrite('log_Xerr_Configuration.csv',log_Xerr);

figure
plot(log_Xerr);
title('Xerr for the end-effector configuration, Kp = , Ki = ','fontsize',16)
xlabel('Time (timestep is set at 10 ms)','fontsize',14) 
ylabel('All six Xerr Values','fontsize',14) 

S = stepinfo(log_Xerr); % Rise time, settling time, and other step-response characteristics

text = 'end of program';
disp(text);

