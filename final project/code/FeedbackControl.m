% Milestone 3: Feedforward Control
% FeedbackControl

%{
% function test 
T = [0 0 0 0 0 0.2 -1.6 0]; % robot configuration
X = [0.170 0 0.985 0.387; 0 1 0 0; -0.985 0 0.170 0.570; 0 0 0 1];
Xd = [0 0 1 0.5; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
Xd_next = [0 0 1 0.6; 0 1 0 0; -1 0 0 0.3; 0 0 0 1];
Kp = zeros(6,6);
Ki = Kp;
timestep = 0.01;
Xerr_integral = zeros(6,1); % initilization of the integral of the error
%}

function Twistee_Integralofeorror ...
         = FeedbackControl(X, Xd, Xd_next, Kp, Ki, timestep, Xerr_integral)
% output return Twist, Xerr, and the integral of the error
% define Input and Output

% Input:

% The current actual end-effector configuration X (also written Tse).
% The current end-effector reference configuration Xd (i.e., Tse,d).
% The end-effector reference configuration at the next timestep in the reference trajectory, Xd,next (i.e., Tse,d,next), at a time ?t later.
% The PI gain matrices Kp and Ki.
% The timestep ?t between reference trajectory configurations.

% Output:

% The commanded end-effector twist \mathcal{V} expressed in the end-effector frame {e}.
% the output contains the command twist and the integral of the error 


% The function called during calculation are also documented as below:

% invT = TransInv(T)
% Takes a transformation matrix T. Returns its inverse. 

% se3mat = MatrixLog6(T)
% Takes a transformation matrix T in SE(3). Returns the corresponding se(3) representation of exponential 

% V = se3ToVec(se3mat)
% Returns the 6-vector twist corresponding to an se(3) matrix se3mat.

% AdT = Adjoint(T)
% Takes T a transformation matrix SE3. 
% Returns the corresponding 6x6 adjoint representation [AdT].


% V(t) = [Adx^-1xd]Vd(t)+KpXerr(t)+Ki*Xerr_integral(t)

Vd_se3 = (1/timestep)*MatrixLog6(TransInv(Xd)*Xd_next);
V = se3ToVec(Vd_se3);
feedforward_term = Adjoint(TransInv(X)*Xd)*V;

Xerr_se3 = MatrixLog6(TransInv(X)*Xd);
Xerr = se3ToVec(Xerr_se3);
Kp_term = Kp*Xerr;

Xerr_integral = Xerr_integral+Xerr*timestep; % integral of all Xerr update
Ki_term = Ki*Xerr_integral;

command_twist= feedforward_term + Kp_term + Ki_term;
Twistee_Integralofeorror = [command_twist Xerr Xerr_integral]; 
% output return Twist, Xerr, and the integral of the error
end