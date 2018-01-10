# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program


---
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

* Gnuplot-iostream. 
  * You can download gnuplot on Mac: ```brew install gnuplot```
  * For the real-time plot, [gnuplot-iostream](http://stahlke.org/dan/gnuplot-iostream/) is utilized. To retireve the respository, 
  ```
  git clone https://github.com/dstahlke/gnuplot-iostream.git
  ```
  * To compile the [gnuplot-iostream.h](./src/gnuplot-iostream.h), [CMakeLists.txt](./CMakeLists.txt) should be updated to include boost and link such as [How do you add boost libraries in CMakeLists.txt](https://stackoverflow.com/questions/6646405/how-do-you-add-boost-libraries-in-cmakelists-txt) in stackoverflow.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Project Rubrics

[//]: # (Image References)
[image1]: ./output/stateequation.png "state_equation"

1. **Compilation**: Your code should compile.

Yes! It is complied with ```cmake``` and ```make```. The ```CMakeLists.txt``` is updated with ```gnuplot-iostream``` to plot the ```cte```, ```epsi```, ```steering angle```, ```throttle```, and ```velocity``` time-histories as real-time.

2. **Implementation**: 

* The model: Student describes their model in detail. This includes the state, actuators and update equations.

The kinematic model of the vehicle in the simulation is built with two element of actuators, steering angle(```delta```) and throttle(```a```).
Also, the state of the model includes ```x``` and ```y```-coordinates, velocity(v), and orientation(```psi```), and cross-track error(```cte```) and orientation error(```epsi```).

State-update equation of the model is given as

![alt text][image1]

* Timestep Length and Elapsed Duration (N and dt): Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The value of ```N``` is tuned according to the reference velocity in the simulation. When the reference velocity is relatively higher, the N should be shorter.
In the first simulation, I set the reference velocity as 30 mph, and the ```N``` is 20. In the last simulation, I choose 70 mph as reference velocity, and ```N``` becomes 15, mentioned in [mpc.cpp](./src/mpc.cpp).

```dt``` is chosen as the value of the latency. Because of the unstable input with delay, we can not accurately predict trajectory of vehicle in smaller time-step than the latency. So, in our case, ```dt``` equals to 0.1.

* Polynomial Fitting:  A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described [main.cpp](./src/main.cpp).

The global coordinates of waypoints are transfromed to vehicle's coordinates. The preprocessed waypoints' coordinate is fitted by 3rd order polynomials.

* Model Predictive Control with Latency: The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

The latency, a 100 milliseconds, is handled with the state-updating. Therefore, we can update the state with 0.1 secons and then we can apply MPC on the delayed state in [main.cpp](./src/main.cpp).


**Here is the simulation result [video](./output/MPC.mov)**

  



