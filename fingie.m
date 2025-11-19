% BME 411 PROJECT SCRIPT: ALLEGRO GRASPING A PENCIL
% This script implements a non-linear 3D grasp optimization problem.
% 1. Loads the Allegro Hand model.
% 2. Defines a cylinder object (a "pencil").
% 3. Maps the object to a pre-defined tripod grasp.
% 4. Extracts the key matrices (G and J) needed for YALMIP.

close all
clear all
clc

disp('--- Setting up Allegro Hand and Grasp ---');

%% 1. HAND DEFINITION
% Load the SGallegroLeft hand model with a default initial pose
T_hand_ini=[SGrotz(pi/2)*SGrotx(pi/2)*SGroty(pi/2) [0 0 0]'; 0 0 0 1];
hand = SGallegroLeft(T_hand_ini);

%% 2. DEFINE GRASP POSE
% This is the 1x16 joint vector for a 3-finger tripod grasp, made to hold a
% scalpel-ish
 q_tripod_grasp=[-0.0636218942771171  0.589117535092792    0.662493612926957    1.05353971973754,...
     0.106768718364394 0.552122034736223   0.736660761567475   1.01601692082632,...
     0.0703882975604282  0.212307167443452   0.897472667580279   0.264856780511055,...
     1.46588154499858  0.124255936445485   -0.180496642956545    1.62042401388558];

q_scalpel_grasp=[-0.0636218942771171  0.589117535092792    0.662493612926957    1.05353971973754,...
    0.106768718364394 0.552122034736223   0.236660761567475   1.01601692082632,...
    0.1703882975604282  0.512307167443452   0.736660761567475   1.01601692082632,...
    1.66588154499858  0.124255936445485   0.380496642956545    1.02042401388558];

%% MOVE TO GRASP POSITION
hand=SGmoveHand(hand,q_tripod_grasp'); % Move to the pose
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


%hand = SGaddContact(hand,1,4,4,1); % thumb
%hand = SGaddContact(hand,1,3,4,1);
%hand = SGaddContact(hand, 1, 1, 3, 0.5)
%hand = SGaddContact(hand, 1, 3, 3, 0.5)



% --- THIS IS THE CORRECT FUNCTION TO USE ---
% It generates an object that fits the contacts you just defined
[hand,obj] = SGmakeObject(hand);

% Plot the resulting object and contacts
SGplotObject(obj)
hold on
SGplotContactPoints(hand,10,'o')

% --- THIS IS YOUR GOAL ---
% Extract the matrices for your optimization
Hand_Jacobian = hand.J;
Grasp_Matrix = obj.G;

disp('--- Successfully Extracted Matrices ---');
disp('Grasp_Matrix (obj.G):');
disp(Grasp_Matrix);
disp('Hand_Jacobian (hand.J):');
disp(Hand_Jacobian);