[//]: # (Image References)
[image1]: ./Data/MPC.png

# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---
## Overview
MPC (Model Predictive Control) method is used as an alternative to the classic PID controller. From the environment and the vehicle state, we know the trajectory that the vehicle should follow. Based on the kinematic vehicle model, we can predict the the vehicle trajectory. Our main role is to select the best steering angle and acceleration value that make the difference between the predicted trajectory and the reference as low as possible.

Using the reference trajectory points (x,y), we can define a polynomial equation to represent the the vehicle path. In this project I used a 3rd degree polynomial equation. 1st degree equation wasn't helpful in road curves and 2nd degree equation wasn't helpful in complex parts of the road that has 2 consecutive curves.

MPC is an optimization problem so we define the vehicle model, the cost function, the vehicle states and actutators and the vehicle constraints.

### Vehicle States and Actuators

Vehicle states are:
```
- X_Position
- Y_Position
- Orientation Angle
- Velocity
- Cross track error
- Orientation error
```

The Vehicle Actuators are:
```
- Steering Angle
- Acceleration
```

### Vehicle Model

The vehicle model is used to predict the next vehicle state as following:

![alt text][image1]

### Cost Function 

The cost function helps to get us the most fit trajectory to the reference. The cost function includes:
```
- Cross track error 
- Orientation error
- Velocity deviation from target velocity (40 mph)
- Steering angle
- Acceleration 
- Steering angle change
- Acceleration change 
```

### Timestep and Elapsed Duration

```
N = 10
dt = 0.1
```
I used higher vlaues for N `15 ,20 ,25` but due to the increase of the number of timesteps the cost function wasn't able to reduse the error which affected the vlaues of steering angle in curves. 

Also using 0.1 for elapsed duration was better than `0.05 & 0.08` due to the `100 ms` latency. 

Check the result from [Here](https://github.com/AhmedMYassin/Model-Predictive-Control/blob/master/Data/result.mp4).

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


