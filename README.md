# pinch-posture-optimization
repo for BME411 group project

Example_1 and Example_2 contain the examples implemented in Mika's paper
Note that Example_2 output does not align with paper -- safety factor is higher because I didn't implement the soft finger model

Run Grip_Optimizer to run the general optimizer. Play with W_ext, which describes the external force acting on the object, to see the different loading conditions. 

Run Plot_Allegro_Hand.m to generate cool visuals. This model was where I generated the Jacobian, Grasp Matrix, and object contact point definitions that are stored in Allegro_data.mat and get called when you run the Grip_Optimizer.

Get_rotation_matrixes is a little helper function that turns the normal vectors output by the allegro hand model, computes their dot product, and saves the tangent vectors to do friction analysis on.  