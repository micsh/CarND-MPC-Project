# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---


## Overview
The goal of this project is to develop a Model Predictive Controller in C++, to steer a car around a track in the simulator. The simulator provides telemetry values containing the position of the car, its speed and orientation. it also provides the coordinates of waypoints along the track that the car needs to follow. The solution uses the IPOPT and CPPAD libraries to calculate an optimal solution in order to minimize the error. This solution plans a path, and tries to follow it by minizing the error from the planned to the actual.

---
## Vehicle Model
The vehicle model used in this project is a kinematic model. It neglects all dynamic effects such as inertia, friction and torque. The model uses the following equations:

    x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] / Lf * delta[t] * dt

Here, **x** and **y** denote the position of the car, **psi** the orientation, **v** the velocity, **cte** the cross-track error and **epsi** the orientation error. **Lf** is the distance between the center of mass of the vehicle and the front wheels and affects the maneuverability. The vehicle model can be found in the class FG_eval.

---
## Pipeline
The waypoints coordinates are transformed:

    dx = ptsx[i] - x
    dy = ptsy[i] - y
    X = dx * cos(psi) - dy * sin(psi)
    Y = dy * cos(psi) + dx * sin(psi)

where **X** and **Y** denote coordinates in the vehicle coordinate system. Next state is therefore calculated intially as such:

    state << 0, 0, 0, speed, cte, epsi

where **cte** and **epsi** are calculated using a **polyfit** evaluation on the transformed values.

## Latency
But when accounting for latency in the system '**dt**', we add a little prediction and get:

    psi = -speed * steer_value * dt / Lf
    state << speed * dt, 0, psi, speed + throttle * dt, cte + speed * sin(epsi) * dt, epsi + psi

The algorithm produces a projection of what to do in the next steps, so to deal with latency we need to incorporate the projected affect of the latency on our state.

---
## Lookahead and time-horizon
The time **_N*dt_** defines the prediction time-horizon. A shorter time-horizon leads to a faster responding controler, but falls short in the projection planning and often finds it difficult to stay stable. Longer prediction time-horizon leads to a better behavior, but too long and it will not react quick enough to difference between predicted/project and actual. I have found a nice spot with N=25 and dt=0.1, which evalutes to an ahead-projection of 2.5 seconds, which was just about right for a good control of the vehicle and enough planning ahead. I have found that below 15 and over 40 provide less optimal results.

---
## Video
[![video](https://img.youtube.com/vi/oCmpK5mJQYk/0.jpg)](https://youtu.be/oCmpK5mJQYk)

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
