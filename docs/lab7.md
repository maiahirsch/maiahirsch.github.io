# Lab 2

## Lab Tasks 

### 1. Estimate drag and momentum

to build the state space model, I have to estimate the drag and momentum acting on my car. Following the derivation from lab, the dynamics of the system can be expressed as: 

$$
\ddot{x} = -\frac{d}{m} \dot{x} + \frac{u}{m}
$$

If we consider a step response from rest to a steady state velocity, acceleration is zero once this velocity is achieved. We can then find d and m: 

$$
d = \frac{u_{ss}}{\dot{x}_{ss}}
$$

$$
m = \frac{-d \cdot t_{0.9}}{ln(1-d \cdot \dot{x}_{ss})} = \frac{-d \cdot t_{0.9}}{ln(1-0.9)}
$$

Where d and m are lumped parameters that capture the dynamics of the system. Showing how the car responds to different inouts and moves throughout the world. 

$u_{ss}$ is the constant control input passed to the robot. We will choose this input to be 1, as this corresponds to the maximum pwm signal possible (255). This is because we want the dybnamics to be as accurate as possible in order to obtain ideal behavior of an accurate controller. 


**To build the state space model for your system, you will need to estimate the drag and momentum terms for your A and B matrices. Here, we will do this using a step response. Drive the car towards a wall at a constant imput motor speed while logging motor input values and ToF sensor output.**

**- Drag terms for A matrix:**
**- Drag terms for B matrix: **

Choose your step responce, u(t), to be of similar size to the PWM value you used in Lab 5 (to keep the dynamics similar). Pick something between 50%-100% of the maximum u.

Make sure your step time is long enough to reach steady state (you likely have to use active braking of the car to avoid crashing into the wall). Make sure to use a piece of foam to avoid hitting the wall and damaging your car.

Show graphs for the TOF sensor output, the (computed) speed, and the motor input. Please ensure that the x-axis is in seconds.

Measure the steady state speed, 90% rise time, and the speed at 90% risetime. Note, this doesn’t have to be 90%, you could also use somewhere between 60-90, but the speed and time must correspond to get an accurate estimate for m.

When sending this data back to your laptop, make sure to save the data in a file so that you can use it even after your Jupyter kernel restarts. Consider writing the data to a CSV file, pickle file, or shelve file.


**2. Initialize KF (Python)**

Compute the A and B matrix given the terms you found above, and discretize your matrices. Be sure to note the sampling time in your write-up.

Ad = np.eye(n) + Delta_T * A  //n is the dimension of your state space 
Bd = Delta_t * B
Identify your C matrix. Recall that C is a m x n matrix, where n are the dimensions in your state space, and m are the number of states you actually measure.
This could look like C=np.array([[-1,0]]), because you measure the negative distance from the wall (state 0).
Initialize your state vector, x, e.g. like this: x = np.array([[-TOF[0]],[0]])

For the Kalman Filter to work well, you will need to specify your process noise and sensor noise covariance matrices.
Try to reason about ballpark numbers for the variance of each state variable and sensor input.
Recall that their relative values determine how much you trust your model versus your sensor measurements. If the values are set too small, the Kalman Filter will not work, if the values are too big, it will barely respond.
Recall that the covariance matrices take the approximate following form, depending on the dimension of your system state space and the sensor inputs.
sig_u=np.array([[sigma_1**2,0],[0,sigma_2**2]]) //We assume uncorrelated noise, and therefore a diagonal matrix works.
sig_z=np.array([[sigma_3**2]])
3. Implement and test your Kalman Filter in Jupyter (Python)
To sanity check your parameters, implement your Kalman Filter in Jupyter first. You can do this using the function in the code below (for ease, variable names follow the convention from the lecture slides).
Import timing, ToF, and PWM data from a straight run towards the wall (you should have this data handy from lab 5).
You may need to format your data first. For the Kalman Filter to work, you’ll need all input arrays to be of equal length. That means that you might have to interpolate data if for example you have fewer ToF measurements than you have motor input updates. This should also be handy from lab 5.
Loop through all of the data, while calling the Kalman Filter.
Remember to scale your input from 1 to the actual value of your step size (u/step_size).
Plot the Kalman Filter output to demonstrate how well your Kalman Filter estimated the system state.
If your Kalman Filter is off, try adjusting the covariance matrices. Discuss how/why you adjust them.
Be sure to include a discussion of all the paramters that affect the performace of your filter.
def kf(mu,sigma,u,y):
    
    mu_p = A.dot(mu) + B.dot(u) 
    sigma_p = A.dot(sigma.dot(A.transpose())) + Sigma_u
    
    sigma_m = C.dot(sigma_p.dot(C.transpose())) + Sigma_z
    kkf_gain = sigma_p.dot(C.transpose().dot(np.linalg.inv(sigma_m)))

    y_m = y-C.dot(mu_p)
    mu = mu_p + kkf_gain.dot(y_m)    
    sigma=(np.eye(2)-kkf_gain.dot(C)).dot(sigma_p)

    return mu,sigma
4. Implement the Kalman Filter on the Robot
Integrate the Kalman Filter into your Lab 5 PID solution on the Artemis. Before trying to increase the speed of your controller, use your debugging script to verify that your Kalman Filter works as expected. Make sure to remove the linear extrapolation step before doing this. Be sure to demonstrate that your solution works by uploading videos and by plotting corresponding raw and estimated data in the same graph.

The following code snippets give helpful hints on how to do matrix operations on the robot:

#include <BasicLinearAlgebra.h>    //Use this library to work with matrices:
using namespace BLA;               //This allows you to declare a matrix

Matrix<2,1> state = {0,0};         //Declares and initializes a 2x1 matrix 
Matrix<1> u;                       //Basically a float that plays nice with the matrix operators
Matrix<2,2> A = {1, 1,
                 0, 1};            //Declares and initializes a 2x2 matrix
state(1,0) = 1;                    //Writes only location 1 in the 2x1 matrix.
Sigma_p = Ad*Sigma*~Ad + Sigma_u;  //Example of how to compute Sigma_p (~Ad equals Ad transposed) 
5. Speed it up (optional)
If you have time, and want to get a jump start on Lab 8, try speeding up your robot with your KF to decrease the execution time of your control loop. Note: you built your Kalman Filter around a specific setpoint u, if you speed up your robot, you will want to check that your model is still valid at the higher higher operating condition.


