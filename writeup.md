[//]: # (Image References)
[image1]: ./media/last_frame.jpg "Result Distance"

**Model Documentation**

## 1. Project structure
Path planning part of the project consists of Planner (planner.cpp, planner.h) and set of States (states.cpp, states.h).
Planner performs calculations of next points for car to follow.
States implement Finite State Machine to switch between different behaviours, in this case between keeping the same lane and changing it.

Planner gets information about the world and gives it to States. States may update Planners parameters. 

## 2. Trajectory selection
Trajectory is selected using cubic spline interpolation which uses set of points to build a polynomial function.
Planner takes all previously built path points, adds points 30, 60 and 90 meters ahead and build spline using this set.

Planner uses three coordinate systems:

* global coordinate system as map is given in it and result points must use it
* frenet system, it is convenient to get points several meters ahead in the lane center
* local car's coordinate system to calculate the spline

When the spline is calculated, Planner supplement previously built path with new points using that spline.

So, these are steps:

* take previous path in global coordinates
* add points 30, 60 and 90 meters ahead, converting them from frenet to global coordinate system
* convert all points to car's local system
* calculate spline
* supplement already built path with new points using reference velocity and spline
* convert all built points back to global coordinate system

This is done in planner.cpp, Predict() method. Planner code is enough to keep the lane and drive with required velocity, acceleration and jerk.
But it doesn't take into account other cars.

## 2. Lane changing

If the lane isn't free and some car is too slow our car should brake or change the lane. 
This logic is done using finite state machine.

It uses three states:

* Keep Lane. If lane is free, then use it and drive with maximal allowed speed.
If lane is occupied with slow car then switch to Find Lane state.
* Find Lane. If planner is in this state then it needs to find another lane and if no lanes are free then to brake (don't collide car in front) and wait for free neighbour lane.
When free lane is found the switch to Change Lane state. If two free lanes are available, the one with bigger distance to the next car is selected.
* Change Lane. Immediately starts lane changing and waits until it's complete. When changing done switch to Keep Lane state.

To calculate the trajectory for lane changing it's enough to change lane parameter of Planner.

C++ polymorphism is used to make Planner independent of States' logic:
* Planner contains pointer to the base State class and on each step runs it Update() method. Planner doesn't know anything else about steps.
* All states inherit base State class and override Update() method with its own logic
* All states contain pointer to update it parameters (lane and max speed) and to switch it to another state

This implementation allows to add new states and modify logic without updating Planner.

States are implemented in states.cpp and states.h files.

## 3. Results
Car can drive more than five miles without any problems:
![alt text][image1]

It follows the lane and change lanes very smoothly: [video preview](./media/lane_changing.mp4 clip).

## 4. Areas for improvement
* Smarter accelerating. Now acceleration is done by increasing reference velocity.
But that increase is done for each new step which depends on response from simulator. If simulator misses few steps then car accelerates much slower than it's allowed. So, generating new points we need to use acceleration, but this complicates linear model which is used right now.
* Controlling distance to the next car. Now collisions are avoided by setting max speed and braking to it. If next car will stop too fast our car may hit it.
* (Not goal for the project) It seems that other cars don't detect the one we control and my hit from sides or from behind. It's better to predict collisions and avoid them.
 


