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

A Kalman filter is an algorithm that "uses a series of measurements observed over time, containing statistical noise and other inaccuracies, and produces estimates of unknown variables that tend to be more accurate than those based on a single measurement alone" ([wikipedia](https://en.wikipedia.org/wiki/Kalman_filter)).

In practice this consists in predicting a future state based in a transition model (from actual state to future state) that has some noise and model inaccuracies (predict step) and correct this prediction based in a measurement provided by a sensor that also has noise (update state). 

For this particular project the state is represented by position and velocity in an xy plane given by the vector: (px, py, vx, vy). **The predict step** is given by the equation:
`x' = f(x) + u`
Where f(x) is assumed to be a linear function `F*x` and u is the noise of the transition model assumed to be a normal distribution with zero mean (in this case representing the unknown acceleration and other possible inacuracies).
The prediction step also implies the update of the covariance uncertainty matrix `P` of the transition model:
`P' = F*P*Ftrans + Q`
Where Q represents the transition noise.

Then in the **the update step** we will compare the predicted state with the measurement received by the sensor and adjust the predicted state considering the covariance uncertainty matrix and the measurement noise. This process follows the following equations:
`y = z - H*x'`
Where z is the measurement provided by the sensor and H is the matrix that projects the state in the measurement space.
`S = H*P'*Htrans + R`
Where R represents the measurement noise.
`K = P'*Htrans*Sinv`
And finally:
`x = x' + K*y`
`P = (I - K*H)*P'`

This equations assumed linearity, however for the radar the projection of the state in the measurement space is not given by a linear function and hence we used the first order taylor expansion of the projection h(x) which is given by the Jacobian matrix Hj.

To implement the algorithm it is important to initialize some of the variable and matrixes:
- The state x is initialize using the first measurement receive and we initialized the position px and py but set the initial velocity to 0 as we can not infer it from the measurement (although the radar gives us the velocity but it is measure in the direction of the line formed by the sensor and the vehicle and this direction does not have to be the same as the velocity direction and hence it will be as inaccurate to derive it from the radar measurement as to set it to zero).
- The transition matrix F needs the time interval between measurements and hence the first measurement is used to set the initial timestamp:
$$
\left(\begin{array}{cc} 
1 & 0 & dt & 0\\
0 & 1 & 0 & dt\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{array}\right)
$$

- The covariance matrix is initialize with a high value for the velocity as we don't know it:
$$
\left(\begin{array}{cc} 
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 1000 & 0\\
0 & 0 & 0 & 1000
\end{array}\right)
$$

- The projection matrix for the Lidar is fix:
\left(\begin{array}{cc} 
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0
\end{array}\right)
$$

- Whereas for the Radar is the Jacobian that has to calculated in each update step.





## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47)
for instructions and the project rubric.

## Hints and Tips!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.
* Students have reported rapid expansion of log files when using the term 2 simulator.  This appears to be associated with not being connected to uWebSockets.  If this does occur,  please make sure you are conneted to uWebSockets. The following workaround may also be effective at preventing large log files.

    + create an empty log file
    + remove write permissions so that the simulator can't write to log
 * Please note that the ```Eigen``` library does not initialize ```VectorXd``` or ```MatrixXd``` objects with zeros upon creation.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! We'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Regardless of the IDE used, every submitted project must
still be compilable with cmake and make.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

