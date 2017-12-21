# CarND-Controls-MPC
---
## Description

This repository holds the source code for a model predictive controller (MPC) used to steer a car around the lake track in Udacity's Self Driving Car Term 2 Simulator.  

---
## Rubric Points

#### <b>1. The Model</b>: Student describes their model in detail. This includes the state, actuators and update equations.

The MPC in this repository utilizes a kinematic model to predict the vehicle's state over time.  The model uses the vehicle's current state and actuator values, namely the x and y coordinates (x, y), orientation angle (Ψ), velocity (v), steering angle (δ), and acceleration (a) along the distance between the front of the vehicle and the it's center of gravity (L<sub>f</sub>) to calculate the vehicle's state change over time including the vehicle's cross track error (cte) and error in psi (eΨ).  The following equations are used to calculate state<sub>t+1</sub> value given state<sub>t</sub>:
<i>
* x<sub>t+1</sub> = x<sub>t</sub> + v<sub>t</sub> * cos(Ψ<sub>t</sub>) * Δt
* y<sub>t+1</sub> = y<sub>t</sub> + v<sub>t</sub> * sin(Ψ<sub>t</sub>) * Δt
* Ψ<sub>t+1</sub> = Ψ<sub>t</sub> + v<sub>t</sub> / L<sub>f</sub> * δ<sub>t</sub> * Δt
* v<sub>t+1</sub> = v<sub>t</sub> + a<sub>t</sub> * Δt
* cte<sub>t+1</sub> = f(x<sub>t</sub>) - y<sub>t</sub> + v<sub>t</sub> * sin(epsi<sub>t</sub>) * Δt
*  eΨ<sub>t+1</sub> = Ψ<sub>t</sub> -  Ψdes<sub>t</sub> + v<sub>t</sub> / L<sub>f</sub> * δ<sub>t</sub> * Δt
</i>

#### <b>2. Timestep Length and Elapsed Duration (N & dt)</b>: Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

N and dt were chosen in this MPC through trial and error.  At higher values of N and/or lower values of dt, the vehicle acted erratically.  At lower values N and/or higher values of dt, the vehicle drove too safe, nearly stopping around corners.  N was tested at values ranging from 5 to 30 and dt at values ranging from 0.05 to 0.2.  The final values were N = 10 and dt = 0.1 which caused the vehicle to smoothly travel the entire track.

#### <b>3. Polynomial Fitting and MPC Preprocessing</b>: A polynomial is fitted to waypoints.  If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

The simulator passes waypoint coordinates to the MPC application.  These waypoints are preprocessed by transforming them to the vehicle's perspective setting the origin to the vehicle's x, y coordinate and rotating the system so the vehicle's orientation is also 0.  This can be found in lines 110 - 116 in main.cpp.  These shifted coordinates are then fitted to a 3rd degree polynomial using the polyfit method in lines 48 - 62 in main.cpp.

#### <b>4. Model Predictive Control with Latency</b>: The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

Prior to fitting the polynomial, the vehicle's current state was shifted to t+1 using the equations above to take vehicle command latency into account. This state change can be found in lines 101 - 107 of main.cpp.  After latency is taken into account, the waypoint points are translated to vehicle's new latency shifted perspective, and the new waypoints are passed into the polynomial fit function to get the desired trajectory's 3rd degree polynomial, the coefficients from the polynomial are used to get the starting cte and eΨ to be used along with the new velocity to make up the vehicle's starting state to be used by the MPC.  This state and the 3rd degree polynomial are then passed to the MPC::solve() function to calculate the next steering value and throttle value to be sent back to the simulator's actuators.  This is done by optimizing actuator inputs at each dt over our N states to minimize a cost function.  The total cost to be minimized is defined as the sum of a number of different cost functions related to the actuator changes and system state over time.  These functions include the cross track error, error in psi, distance to desired velocity, actuator use, and magnitude of actuator change between two states.  Each separate cost function was then further tuned by adding a multiplier to the function depending as it relates to that specific cost functions magnitude of importance.  For instance, the cost function for the cross track error and error in psi have very large multipliers compared to the other functions because above all else, we want the vehicle to stay on the road.  All functions' magnitudes were tuned first by intuition and then further by trial and error.

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


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
