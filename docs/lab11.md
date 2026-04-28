# Lab 11: Localization on the real robot 

## Task 1: Test Localization in Simulation



Test Localization in Simulation: Run the notebook lab11_sim.ipynb and attach a single screenshot of the final plot (odom, ground truth and belief).
Using a uniform prior on the pose, run (only) the update step using the sensor measurement data to localize your robot
Go through the notebook lab11_real.ipynb and implement the member function perform_observation_loop of class RealRobot (re-use code from previous labs to implement this).
Place your robot in one of the four marked poses and run the update step of the Bayes filter once.
How close is the localized pose w.r.t to the ground truth?
Visualize your results
Discuss your results
Repeat (2) for every marked position.
Does the robot localize better in certain poses? If so, why?


