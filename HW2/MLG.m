function [Rot ,Pot] = fwdkin_Dofbot (q)
% FWDKIN_DOFBOT Computes the end effector position and orientation relative
%to the base frame for Yahboom ’s Dofbot manipulator using the product of
% exponentials approach
% Input :
% q: 5x1 vector of joint angles in degrees
%
% Output :
% Rot: The 3x3 rotation matrix describing the relative orientation of
% the end effector frame to the base frame (R_ {0T})
% Pot: The 3x1 vector describing the position of the end effector
% relative to the base , where the first element is the position
% along the base frame x-axis , the second element is the position
% along the base frame y-axis , and the third element is the
% position along the base frame z- axis (P_ {0T})

%% Set up the basis unit vectors
ex = [1;0;0]; %x axis
ey = [0;1;0]; %y axis
ez = [0;0;1]; %z axis

%% Define the link lengths in meters
l0 = 0.061; % base to servo 1
l1 = 0.0435; % servo 1 to servo 2
l2 = 0.08285; % servo 2 to servo 3
l3 = 0.08285; % servo 3 to servo 4
l4 = 0.07385; % servo 4 to servo 5
l5 = 0.05457; % servo 5 to gripper

%% set up the rotation matrices between subsequent frames
% for MATLAB, an element of a vector is extracted using parentheses
% for MATLAB, the indexing starts at 1 (the first element of q is q(1))
% (in Python indexing starts at 0)
R01 = rotz (q(1)); % rotation between base frame and 1 frame
R12 = roty (-q(2)); % rotation between 1 and 2 frames
R23 = roty (-q(3)); % rotation between 2 and 3 frames
R34 = roty (-q(4)); % rotation between 3 and 4 frames
R45 = rotx (-q(5)); % rotation between 4 and 5 frames
% look up "MATLAB identity matrix" to find the function missing here. This line will return the 3x3 identity matrix
R5T = eye(3) ; %the tool frame is defined to be the same as frame 5

%% set up the position vectors between subsequent frames
P01 = (l0+l1)*ez; % translation between base frame and 1 frame in base frame
% the function zeros(m,n) will return a matrix of zeros with m rows and n columns
P12 = zeros(3 ,1); % translation between 1 and 2 frame in 1 frame
P23 = l2*ex; % translation between 2 and 3 frame in 2 frame
P34 = -l3*ez; % translation between 3 and 4 frame in 3 frame
P45 = zeros(3 ,1); % translation between 4 and 5 frame in 4 frame
P5T = -(l4+l5)*ex; % translation between 5 and tool frame in 5 frame

%% calculate Rot and Pot
%Rot is a sequence of rotations
Rot = R01*R12*R23*R34*R45*R5T;
%Pot is a combination of the position vectors . Each vector must be
% represented in the base frame before addition . This is achieved using the
% rotation matrices .
Pot = P01 + R01 *( P12 + R12 *( P23 + R23 *( P34 + R34 *( P45 + R45*P5T))));


    function r = rotz(q)
       %% create the z axis rotation matrix
       % cosd is the cosine function that accepts input in degrees (cos is for radians input)
       % sind is the sine function that accepts input in degrees (sin is for radians input)
        r = [cosd(q) -sind(q) 0; sind(q) cosd(q) 0; 0 0 1];
    end

    function r = roty(q)
        %% create the y axis rotation matrix
        r = [cosd(q) 0 sind(q);0 1 0; -sind(q) 0 cosd(q)];
    end

    function r = rotx(q)
        %% create the x axis rotation matrix
        r = [1 0 0; 0 cosd(q) -sind(q);0 sind(q) cosd(q)];
    end

end
