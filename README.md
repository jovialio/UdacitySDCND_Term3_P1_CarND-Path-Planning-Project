# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---
## Reflection

This project had mostly made use of the topic on **Behaviour Planning** and **Trajectory Generation**. Many thanks to the Udacity team for their guidance on the project especially on the use of spline library for trajectory generation using waypoints forecasted. There are still room for improvements by better planning for more aggressive but safe lane changes such as speeding up for change lane instead of slowing down to wait for target lane vehicle to pass before changing lanes. This implementation is safe but penalizes speed of completing the circuit as it ocassionally leads to difficulties changing lanes. Overall, the implementation in it current state prioritizes *safety* followed by *speed* in completing the circuit.

Below is a short summary of my implementation for the project that covers the major rubrics demanded of the project. Code in `main.cpp` are also commented to provide explanation.

## Implementation

For ease of reading, we shall call the Autonomous Vehicle *ego*. ;)

1. The code compiles correctly.
    1. Code is able to comple with cmake.

2. The car is able to drive at least 4.32 miles without incident.
    1. ego is able to meet specifications.

3. The car drives according to the speed limit.
    1. Overall speed limit for vehicle is set to 49.5MPH which is the limit of the road. While travelling, speed limit on the 3 lanes are dynamically calculated based on the speed of the immediate vehicle in each lane closest to ego.

4. Max Acceleration and Jerk are not Exceeded.
    1. Max Acceleration and Jerk are normally not exceed. Exceptions is deliberatly added in the case of emergency breaking by the vehicle infront of ego where projected future s of vehicle in front and ego will come to <10m (Usual `SAFETY_DISTANCE` is set to 30m). In this case, reference velocity will be suddenly decreased by 2MPH(0.894m/s) or subjected to a deceleration of 44.7m/s2 in 0.02s interval due the the plot points being spaced by 0.02s interval.

5. Car does not have collisions.
    1. ego does not have any collision by maintaining a front vehicle safety margin of 30m and back vehicle safety margin of 20m during considerations for lane change. This reduction in safety margin for back vehicle allows easier lane changing as long as ego is 20m cleared of rear vehicle in the target lane. Lane keeping only maintains a 30m `SAFETY_DISTANCE` from front vehicle.

6. The car stays in its lane, except for the time between changing lanes.
    1. ego's priority is safety followed by attempting to complete the circuit in the fastest time. ego will change lane as long as it detects a potentially faster lane other than its own lane. This is the only time when ego will consider making a lane change safely.

7. The car is able to change lanes.
    1. Yes, ego is able to change lanes.

8. There is a reflection on how to generate paths.
    1. Frenet s,d coordinates is used most of the time during planning for checking ego's current position (s,d) and other vehicles' (s,d). Frenet coordinates allows better gauge of ego's location versus other vehicle in order to effect a safe Keep Lane (KL), Lane Change Left (LCL) or Lane Change Right (LCR), which are the 3 simple states in the simulation.
    2. Once the state is determined, 3 more waypoints in xy coordinates will be determined taking into considerations the chosen s (distance ahead) and d (lane choice). These 3 wayspoints will be picked roughly in 30m interval from ego's current position or previously planned path s position.
    3. The 3 xy coordinate waypoints will be then combined with either the last 2 points from previously planned path or 2 newly generated points based on ego's current orientation.
    4. Using these 5 xy points, spline library will be used to generate the remaining points in between to make up a total of 50 points for ego's trajectory.
        1. To generate these 50 points, spline library will be used on xy coordinates converted from map space to car space by using a rotation matrix and ego is at the centre (0,0) position.
        2. Suitable number of points will be filled in between the way points taking into consideration the current reference velocity (ref_v) and the uniqueness of simulator which drops points in 0.02s interval.
        3. Once 50 points are generated, they will be pushed to `next_x_vals` and `next_y_vals` for input into simulator for execution.

## Path Planning Video

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/LQYuXH4gVlk/0.jpg)](https://www.youtube.com/watch?v=LQYuXH4gVlk)

---

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

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


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
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
