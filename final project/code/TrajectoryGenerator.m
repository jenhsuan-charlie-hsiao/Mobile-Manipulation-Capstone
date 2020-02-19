function Ttraj ...
         = TrajectoryGenerator(Tse_initial, Tsc_initial, Tsc_fianl, ...
                                     Tce_grasp, Tce_standoff, k, a)

%% %% mae204 FINAL PROJECT
%addpath(genpath('mr')) %include Modern Robotics library

%Tse_initial = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0.4012; 0, 0, 0, 1];
% initial configuratino of the end effector in the reference trajectory

%Tsc_initial = [1, 0, 0, 1; 0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
%the cube's initial configuration

%Tsc_fianl = [0, 1, 0, 0; -1, 0, 0, -1; 0, 0, 1, 0; 0, 0, 0, 1];
%the cube's final configuration

%Tce_grasp = [0, 0, 1, 0; 0, 1, 0, 0; -1, 0, 0, 0.025; 0, 0, 0, 1];
%configuration of the end effector {e} relative to the cube frame {c} when
%the cube is held by the gripper

%Tce_standoff = [0, 0, 1, 0; 0, 1, 0, 0; -1, 0, 0, 0.225; 0, 0, 0, 1];
%end effector's standoff configuration above the cube, before and after
%grasping, relative to the cube

%k = 1;
%The number of reference configurations per 0.01 seconds: k
% Num_Configurations = a*sum(Tfi*k/0.01) = a*1000

Te0 = Tse_initial; % gripper state = 0, open
Te1 = Tsc_initial*Tce_standoff; %Tse = Tsc*Tce, gripper state = 0, open
Te2 = Tsc_initial*Tce_grasp; %Tse = Tsc*Tce, gripper state = 0, open
Te3 = Te2; %gripper state = 1, closed
Te4 = Te1; %gripper state = 1, closed
Te5 = Tsc_fianl*Tce_standoff; %gripper state = 1, closed
Te6 = Tsc_fianl*Tce_grasp; %gripper state = 1, closed
Te7 = Te6; % gripper state = 0, open
Te8 = Te5; % gripper state = 0, open


%% traj1, from Te0 to Te1
Tf = 2*a;
N = Tf*(k/0.01); % N reference configurations
method = 3;
%{
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete 
%          representation of the trajectory,
%       method: The time-scaling method, where 3 indicates cubic 
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
%}

Ttraj1 = zeros(N,13);

traj = CartesianTrajectory(Te0, Te1, Tf, N, method);
  for i = 1: N
    Ttraj1(i,1) = traj{1,i}(1,1);
    Ttraj1(i,2) = traj{1,i}(1,2);
    Ttraj1(i,3) = traj{1,i}(1,3);
    Ttraj1(i,4) = traj{1,i}(2,1);
    Ttraj1(i,5) = traj{1,i}(2,2);
    Ttraj1(i,6) = traj{1,i}(2,3);
    Ttraj1(i,7) = traj{1,i}(3,1);
    Ttraj1(i,8) = traj{1,i}(3,2);
    Ttraj1(i,9) = traj{1,i}(3,3);
    Ttraj1(i,10) = traj{1,i}(1,4);
    Ttraj1(i,11) = traj{1,i}(2,4);
    Ttraj1(i,12) = traj{1,i}(3,4);
    Ttraj1(i,13) = 0; %gripper state
 end
 
% csvwrite('reference_trajectory.csv',Ttraj)

%% traj2, from Te1 to Te2
Tf = 1*a;
N = Tf*(k/0.01);
method = 3;
%{
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete 
%          representation of the trajectory,
%       method: The time-scaling method, where 3 indicates cubic 
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
%}

Ttraj2 = zeros(N,13);

traj = CartesianTrajectory(Te1, Te2, Tf, N, method);
  for i = 1: N
    Ttraj2(i,1) = traj{1,i}(1,1);
    Ttraj2(i,2) = traj{1,i}(1,2);
    Ttraj2(i,3) = traj{1,i}(1,3);
    Ttraj2(i,4) = traj{1,i}(2,1);
    Ttraj2(i,5) = traj{1,i}(2,2);
    Ttraj2(i,6) = traj{1,i}(2,3);
    Ttraj2(i,7) = traj{1,i}(3,1);
    Ttraj2(i,8) = traj{1,i}(3,2);
    Ttraj2(i,9) = traj{1,i}(3,3);
    Ttraj2(i,10) = traj{1,i}(1,4);
    Ttraj2(i,11) = traj{1,i}(2,4);
    Ttraj2(i,12) = traj{1,i}(3,4);
    Ttraj2(i,13) = 0; %gripper state
  end
 
%% traj3, from Te2 to Te3
Tf = 1*a;
N = Tf*(k/0.01);
method = 3;
%{
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete 
%          representation of the trajectory,
%       method: The time-scaling method, where 3 indicates cubic 
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
%}

Ttraj3 = zeros(N,13);

traj = CartesianTrajectory(Te2, Te3, Tf, N, method);
  for i = 1: N
    Ttraj3(i,1) = traj{1,i}(1,1);
    Ttraj3(i,2) = traj{1,i}(1,2);
    Ttraj3(i,3) = traj{1,i}(1,3);
    Ttraj3(i,4) = traj{1,i}(2,1);
    Ttraj3(i,5) = traj{1,i}(2,2);
    Ttraj3(i,6) = traj{1,i}(2,3);
    Ttraj3(i,7) = traj{1,i}(3,1);
    Ttraj3(i,8) = traj{1,i}(3,2);
    Ttraj3(i,9) = traj{1,i}(3,3);
    Ttraj3(i,10) = traj{1,i}(1,4);
    Ttraj3(i,11) = traj{1,i}(2,4);
    Ttraj3(i,12) = traj{1,i}(3,4);
    Ttraj3(i,13) = 1; %gripper state
  end
 
%% traj4, from Te3 to Te4
Tf = 1*a;
N = Tf*(k/0.01);
method = 3;
%{
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete 
%          representation of the trajectory,
%       method: The time-scaling method, where 3 indicates cubic 
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
%}

Ttraj4 = zeros(N,13);

traj = CartesianTrajectory(Te3, Te4, Tf, N, method);
  for i = 1: N
    Ttraj4(i,1) = traj{1,i}(1,1);
    Ttraj4(i,2) = traj{1,i}(1,2);
    Ttraj4(i,3) = traj{1,i}(1,3);
    Ttraj4(i,4) = traj{1,i}(2,1);
    Ttraj4(i,5) = traj{1,i}(2,2);
    Ttraj4(i,6) = traj{1,i}(2,3);
    Ttraj4(i,7) = traj{1,i}(3,1);
    Ttraj4(i,8) = traj{1,i}(3,2);
    Ttraj4(i,9) = traj{1,i}(3,3);
    Ttraj4(i,10) = traj{1,i}(1,4);
    Ttraj4(i,11) = traj{1,i}(2,4);
    Ttraj4(i,12) = traj{1,i}(3,4);
    Ttraj4(i,13) = 1; %gripper state
  end
 
%% traj5, from Te4 to Te5
Tf = 2*a;
N = Tf*(k/0.01);
method = 3;
%{
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete 
%          representation of the trajectory,
%       method: The time-scaling method, where 3 indicates cubic 
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
%}


Ttraj5 = zeros(N,13);

traj = CartesianTrajectory(Te4, Te5, Tf, N, method);
  for i = 1: N
    Ttraj5(i,1) = traj{1,i}(1,1);
    Ttraj5(i,2) = traj{1,i}(1,2);
    Ttraj5(i,3) = traj{1,i}(1,3);
    Ttraj5(i,4) = traj{1,i}(2,1);
    Ttraj5(i,5) = traj{1,i}(2,2);
    Ttraj5(i,6) = traj{1,i}(2,3);
    Ttraj5(i,7) = traj{1,i}(3,1);
    Ttraj5(i,8) = traj{1,i}(3,2);
    Ttraj5(i,9) = traj{1,i}(3,3);
    Ttraj5(i,10) = traj{1,i}(1,4);
    Ttraj5(i,11) = traj{1,i}(2,4);
    Ttraj5(i,12) = traj{1,i}(3,4);
    Ttraj5(i,13) = 1; %gripper state
  end
  
%% traj6, from Te5 to Te6
Tf = 1*a;
N = Tf*(k/0.01);
method = 3;
%{
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete 
%          representation of the trajectory,
%       method: The time-scaling method, where 3 indicates cubic 
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
%}

Ttraj6 = zeros(N,13);

traj = CartesianTrajectory(Te5, Te6, Tf, N, method);
  for i = 1: N
    Ttraj6(i,1) = traj{1,i}(1,1);
    Ttraj6(i,2) = traj{1,i}(1,2);
    Ttraj6(i,3) = traj{1,i}(1,3);
    Ttraj6(i,4) = traj{1,i}(2,1);
    Ttraj6(i,5) = traj{1,i}(2,2);
    Ttraj6(i,6) = traj{1,i}(2,3);
    Ttraj6(i,7) = traj{1,i}(3,1);
    Ttraj6(i,8) = traj{1,i}(3,2);
    Ttraj6(i,9) = traj{1,i}(3,3);
    Ttraj6(i,10) = traj{1,i}(1,4);
    Ttraj6(i,11) = traj{1,i}(2,4);
    Ttraj6(i,12) = traj{1,i}(3,4);
    Ttraj6(i,13) = 1; %gripper state
  end
 
%% traj7, from Te6 to Te7
Tf = 1*a;
N = Tf*(k/0.01);
method = 3;
%{
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete 
%          representation of the trajectory,
%       method: The time-scaling method, where 3 indicates cubic 
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
%}

Ttraj7 = zeros(N,13);

traj = CartesianTrajectory(Te6, Te7, Tf, N, method);
  for i = 1: N
    Ttraj7(i,1) = traj{1,i}(1,1);
    Ttraj7(i,2) = traj{1,i}(1,2);
    Ttraj7(i,3) = traj{1,i}(1,3);
    Ttraj7(i,4) = traj{1,i}(2,1);
    Ttraj7(i,5) = traj{1,i}(2,2);
    Ttraj7(i,6) = traj{1,i}(2,3);
    Ttraj7(i,7) = traj{1,i}(3,1);
    Ttraj7(i,8) = traj{1,i}(3,2);
    Ttraj7(i,9) = traj{1,i}(3,3);
    Ttraj7(i,10) = traj{1,i}(1,4);
    Ttraj7(i,11) = traj{1,i}(2,4);
    Ttraj7(i,12) = traj{1,i}(3,4);
    Ttraj7(i,13) = 0; %gripper state
  end
 
%% traj8, from Te7 to Te8
Tf = 1*a;
N = Tf*(k/0.01);
method = 3;
%{
%       Tf: Total time of the motion in seconds from rest to rest,
%       N: The number of points N > 1 (Start and stop) in the discrete 
%          representation of the trajectory,
%       method: The time-scaling method, where 3 indicates cubic 
%               (third-order polynomial) time scaling and 5 indicates 
%               quintic (fifth-order polynomial) time scaling.
%}

Ttraj8 = zeros(N,13);

traj = CartesianTrajectory(Te7, Te8, Tf, N, method);
  for i = 1: N
    Ttraj8(i,1) = traj{1,i}(1,1);
    Ttraj8(i,2) = traj{1,i}(1,2);
    Ttraj8(i,3) = traj{1,i}(1,3);
    Ttraj8(i,4) = traj{1,i}(2,1);
    Ttraj8(i,5) = traj{1,i}(2,2);
    Ttraj8(i,6) = traj{1,i}(2,3);
    Ttraj8(i,7) = traj{1,i}(3,1);
    Ttraj8(i,8) = traj{1,i}(3,2);
    Ttraj8(i,9) = traj{1,i}(3,3);
    Ttraj8(i,10) = traj{1,i}(1,4);
    Ttraj8(i,11) = traj{1,i}(2,4);
    Ttraj8(i,12) = traj{1,i}(3,4);
    Ttraj8(i,13) = 0; %gripper state
  end
 
  
Ttraj = [Ttraj1; Ttraj2; Ttraj3; Ttraj4; Ttraj5; Ttraj6; Ttraj7; Ttraj8];
 %csvwrite('reference_trajectory.csv',Ttraj)
end