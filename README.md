## SelfDrivingCarND_MPCController

Implementation of a MPC controller with the aim to drive a car in the Simulation environent provided by Udacity in the scope of the Self driving car NanoDegree. Starting from the current car information (position x and y, speed, steering angle and throttle) and the target trajectory points provided by the simulator, the MPC meshes all the informations in an initial state, and optimize the future actuations on throttle and steering, mimymizing the defined cost function.
## Repository structure
The repository is made of two folders and some files:
1.  /src folder: includes all the src files required for executable build and to execute the PID controller;
2.	/executables folder: contains the compiled files
3.  DATA.md shows how informations are provided by the simulator via uWS sockets
4.  lake_track_waypoints.csv provides data used to make a first optimization of the controller parameters.
5.  CMakeLists.txt and cmakepatch.txt are the configuration files used to make the system executable on windows. 
6.  install-mac.sh and install-ubuntu.sh: scripts for mac and ubuntu provided by udacity in order to make the configuration of the OS faster (e.g. uWebSockets installation) 
## 1. Dependencies
* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.
## 2. Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.
## 3. Environment details
The project was developed in the Eclipse Keples environment, under the linux distribution Ubuntu 17.04.
## 4. Controller Skeleton
The skeleton of the PID project is made by two main parts: 
1. *main.cpp* which include all the logic for the simulator connection as well as the management of the computed actuations before the trasmission to the simulator; 
2. MPC.h and MPC.cpp which includes the logic of the controller itself.
The main function has the aim to manage the entire connection with the simulator and to adjust the actuations required by the MPC 
before to transmit these to the simulator for the actuation. In the loop of the main function is also managed the communication with the simulator for the trajectories to be shownd.
## 5. The Model and the Optimization
The MPC model is mainly based on 2 core parts:
1.  The Model of the system;
2.  The cost function to be optimized.
Both the parts are stronly influenced by the 2 parameter `N` and `d_t` which defines how far the controller should see, and how is the time between two timestamp.
### 5.1 The model
The model equations are based on the Kinematic model equations. At each time stamp `t+1`, the data about the model are computed accordngly to the following equations:
```
x_t+1 = x_t + v_t ∗ cos(psi_t) ∗ d_t
y_t+1 = y_t + v_t * sin(psi_t) * d_t
psi_t+1 = psi_t + L_f * v_t * delta_t * d_t
v_t+1 = v_t + a_t ∗ d_t
cte_t+1 = f(x_t) − y_t + (v_t ∗ sin(epsi_t) ∗ d_t)
epsi_t+1 = psi_t − psi_des_t + (L_f * v_t * delta_t* d_t)
```
These equations requires the knowledge of the d_t parameter, that is the time between the last timestamp and the current one.
### 5.2 The cost function
The cost function is the core of the MPC model since is used duting the optimizaion process by the Ipopt library. The solver find the actuations that minimize the cost function, considering the variables and the actuation constraints.
In the current MPC implementation, different elements were taken into consideration for the cost function:
1.  the cross track error (CTE);
2.  the error on the orientation psi (EPSI); 
3.  the distance from the current target speed ref_v
```c++
for (int t = 0;   t < N;  t++) {
  fg[0] += cte_cost_coeff   * CppAD::pow( vars[cte_start  + t] -  ref_cte,  2)
       + epsi_cost_coeff  * CppAD::pow( vars[epsi_start + t] - ref_epsi,  2)
       + speed_cost_coeff * CppAD::pow( vars[v_start 	  + t] -    ref_v,  2) ;
}
```  
4.  the steering value;
5.  the acceleration value
```c++
for (int t = 0;   t < (N - 1);    t++) {
  fg[0] += steering_cost_coeff * CppAD::pow( vars[delta_start + t],  2)
       + throttle_cost_coeff * CppAD::pow( vars[a_start  	  + t],  2) ;
}
```
6.  the change rate of the steering;
7.  the change rate of the acceleration.
```c++
// Add to the cost function the error of delta actuations. Make the actuations smooth
for (int t = 0;   t < (N - 2);   t++) {
  fg[0] += delta_steering_cost_coeff * CppAD::pow( vars[delta_start + t + 1] - vars[delta_start + t],  2)
       + delta_throttle_cost_coeff * CppAD::pow( vars[a_start     + t + 1] - vars[a_start 	+ t],  2) ;
}
```
While the first 5 components create a direct influence of the main important features for the controller (cte, epsi, v, steering value and acceleration), the most important are the last 2 values. These control the MPC respect to the change rate for acceleration and steering angle.
Each component of the cost function are regulated by a dedicated coeficients. These coefficients were the main focus during the optimization phase. 
```c++
cte_cost_coeff = 200;
epsi_cost_coeff	= 50;
speed_cost_coeff = 1;
throttle_cost_coeff = 15;
steering_cost_coeff = 350;
delta_throttle_cost_coeff = 30;
delta_steering_cost_coeff = 7000;
```
## 6. The latency inclusion
Latency is included in the simulator management in order to mimic the real world and the time between the actuation require and the actuation itself. 
This delay is considered by the sistem with a sleep of 100ms
```c++
this_thread::sleep_for(chrono::milliseconds(100));
```
This delay is then included in the computation of the variables for the state vector. A latency parameter of .1 is applied to some of the state values, before to run the MPC Solver.
```c++
Eigen::VectorXd ComputeStateWithLatency(double v, double steer, double throttle, double cte, double epsi){
  	//factor in delay
	double delay_x = v * latency;
	double delay_y = 0;
	double delay_psi = -v*steer / Lf * latency;
	Eigen::VectorXd state(6);
	state << delay_x, delay_y, delay_psi, v, cte, epsi;
	return state;
}
```
The latency is not applied to the entire values in the state vector, but only to the x car position and the psi value.
