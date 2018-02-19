# Carnd Extended Kalman Filter Project

Self-Driving Car Engineer Nanodegree Program

This project consist in the implementation of a kalman filter in C++ and testing it with a simulator provided by Udacity ([download here](https://github.com/udacity/self-driving-car-sim/releases)) that generates noisy Radar an Lidar measurements. The communication between the simulator and the EKF is done using WebSocket using the uWebSockets implementation (this repository includes scripts for Mac Os: install-mac.sh and Ubuntu: install-ubuntu.sh to install uWebSockets).


## Compiling and executing the project

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. Clone the repo and cd to it on a Terminal.
2. (Within the project repo) mkdir build
3. cd build
4. cmake ..
5. make
6. ./ExtendedKF

## Kalman filter for Dummies

A Kalman filter is an algorithm that "_uses a series of measurements observed over time, containing statistical noise and other inaccuracies, and produces estimates of unknown variables that tend to be more accurate than those based on a single measurement alone_" ([wikipedia](https://en.wikipedia.org/wiki/Kalman_filter)).

In practice this consists in predicting a future state based in a transition model (from actual state to future state) that has some noise and model inaccuracies (predict step) and correct this prediction based in a measurement provided by a sensor that also has noise (update state). 

For this particular project the state is represented by position and velocity in an xy plane given by the vector: `[px, py, vx, vy]`. **The predict step** is given by the equation:

`x' = f(x) + u`

Where `f(x)` is assumed to be a linear function `F*x` and u is the noise of the transition model assumed to be a normal distribution with zero mean (in this case representing the unknown acceleration and other possible inaccuracies).
The prediction step also implies the update of the covariance uncertainty matrix `P` of the transition model:

`P' = F*P*Ftrans + Q`

Where `Q` represents the transition noise.

Then in the **the update step** we will compare the predicted state with the measurement received by the sensor and adjust the predicted state considering the covariance uncertainty matrix and the measurement noise. This process follows the following equations:

`y = z - H*x'`

Where z is the measurement provided by the sensor and `H` is the matrix that projects the state in the measurement space.

`S = H*P'*Htrans + R`

Where `R` represents the measurement noise.

`K = P'*Htrans*Sinv`

And finally:

`x = x' + K*y`
`P = (I - K*H)*P'`

This equations assumed linearity, however for the radar the projection of the state in the measurement space is not given by a linear function and hence we used the first order taylor expansion of the projection `h(x)` which is given by the Jacobian matrix `Hj`.

To implement the algorithm it is important to initialize some of the variable and matrixes:
- The state `x` is initialize using the first measurement receive and we initialized the position px and py but set the initial velocity to 0 as we can not infer it from the measurement (although the radar gives us the velocity but it is measure in the direction of the line formed by the sensor and the vehicle and this direction does not have to be the same as the velocity direction and hence it will be as inaccurate to derive it from the radar measurement as to set it to zero).
- The transition matrix `F` needs the time interval between measurements and hence the first measurement is used to set the initial timestamp:  
`1    0   dt    0`  
`0    1    0   dt`  
`0    0    1    0`  
`0    0    0    1`  

- The covariance matrix `P` is initialize with a high value for the velocity as we don't know it:  
`1    0    0    0`  
`0    1    0    0`  
`0    0 1000    0`  
`0    0    0 1000`  

- The projection matrix `H` for the Lidar is fix:  
`1    0    0    0`  
`0    1    0    0`  

- Whereas for the Radar is the Jacobian `Hj` that has to calculated in each update step.


## Kalman filter implementation

This algorithm has been implemented in C++ in the source code files provided in the src directory using the templates provided by Udacity with the following changes:
- **main.cpp**: code for choosing which sensor to use (only Lidar, only Radar or both) has been added in lines 36 to 69.
- **FusionEKF.cpp**: constructor of FusionEKF class (lines 38 to 49), initialization of the process (86 to 102) and predict / update steps (127 to 161) has been implemented plus code to control the sensor to be used (lines 60 to 68).
- **kalman\_filter.cpp**: implementation of methods Predict, Update and UpdateEKF.
- **tools.cpp**: implementation of methods CalculateRMSE and CalculateJacobian.

## Results

Using **both sensors** the results of the RMSE are below the required threshold:  
RMSE:    	0.0973178 0.0854597  0.451267  0.439935

If we try to use just one of the sensors we obtain the following results:  
- **Only Lidar**:  
RMSE:    	 0.122191 0.0983799  0.582513  0.456699

- **Only Radar**:  
RMSE:    	0.191769 0.279809 0.557542 0.656725

This results shows the higher accuracy of the Lidar sensor that provides better position estimation and based in this higher accuracy also provides a better estimate for the velocity although the Radar can measure the velocity. 