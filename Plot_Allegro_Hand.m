
%    M. Pozzi, M. Malvezzi, D. Prattichizzo
%    
%    References:
%    1. D. Prattichizzo, M. Pozzi, M. Malvezzi. Robotic Grasping.
%    In Springer Robotics Goes Mooc, Ed. B. Siciliano, submitted.
%    2. V. Ruiz Garate, M. Pozzi, D. Prattichizzo, Arash A.. A Bio-Inspired 
%    Grasp Stiffness Control for Robotic Hands. Frontiers in Robotics 
%    and AI, 2018. 
%    3. V. Ruiz Garate, M. Pozzi, D. Prattichizzo, N. Tsagarakis, A.  
%    Ajoudani. Grasp Stiffness Control in Robotic Hands through Coordinated  
%    Optimization of Pose and Joint Stiffness. IEEE Robotics and Automation 
%    Letters, 3(4):3952-3959, October 2018.

% close all
% clear all
% clc

%% HAND DEFINITION 
T_hand_ini=[SGrotz(pi/2)*SGrotx(pi/2)*SGroty(pi/2) [0 0 0]';
    0 0 0 1];

hand = SGallegroLeft(T_hand_ini); % SGallegroRight(T_hand_ini);

%% POSSIBLE POSITIONS

q_zero=[0,0,0,0, ... %index
    0,0,0,0, ... %middle
    0,0,0,0, ... %last
    0,0,0,0]; %thumb

q_tripod_grasp=[-0.0636218942771171  0.589117535092792    0.662493612926957    1.05353971973754,...
    0.106768718364394 0.552122034736223   0.736660761567475   1.01601692082632,...
    0.0703882975604282  0.212307167443452   0.897472667580279   0.264856780511055,...
    1.46588154499858  0.124255936445485   -0.180496642956545    1.62042401388558];

q_home=[-0.02785654223134834, 0.009314805919159919, 0.918914391495951, 0.7941311423884578, ...
    0.0066785400981930945, -0.005536158235084111, 0.8715494821394858, 0.764692840660485, ...
    0.06300675323675811, 0.029174675142167622, 0.868913216319781, 0.8087184799534589, ...
    0.7299820072825081, 0.4579193740155175, 0.23919718596737496, 0.7686472393976554];


%% MOVE TO ZERO POSITION
hand=SGmoveHand(hand,q_zero);

% Plot
figure(1) 
SGplotHand(hand);
hold on
xlabel('x')
ylabel('y')
zlabel('z')
title("Initial position")

%% MOVE TO HOME POSITION
hand=SGmoveHand(hand,q_home');

% Plot
figure(2) 
SGplotHand(hand);
hold on
xlabel('x')
ylabel('y')
zlabel('z')
title("Home position")

%% MOVE TO GRASP POSITION
hand=SGmoveHand(hand,q_tripod_grasp');

% Plot
figure(3) 
SGplotHand(hand);
hold on
xlabel('x')
ylabel('y')
zlabel('z')
title("Grasp position")

% Definition of contact points (tripod grasp)
hand = SGaddContact(hand,1,1,4,1); % index
hand = SGaddContact(hand,1,2,4,1); % middle
hand = SGaddContact(hand,1,4,4,1); % thumb

% Define the object
[hand,obj] = SGmakeObject(hand);

% Plot object and contact points
SGplotObject(obj)
hold on
SGplotContactPoints(hand,10,'o')

% Matrices for grasp analysis
Hand_Jacobian = hand.J;
Grasp_Matrix = obj.G;
