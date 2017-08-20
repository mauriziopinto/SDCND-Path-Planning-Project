# SDCND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Summary

This repository contains a C++ implementation of a path planner to be used with the Udacity simulator available at https://github.com/udacity/self-driving-car-sim/releases. It has been implemented as part of the Udacity Self-Driving Car Engineer Nanodegree Program.

## Goal of the program

The path planner implements a simple behaviour:

* keep the current lane at a cruise speed of about 50 MPH
* if the current lane is occupied by a slower car, evaluate a lane change and overtake the slow car
* if changing lane is not possible, stay in the current lane and set the cruise speed to the one of the car ahead

All manuevers should not exceed a maximum jerk, in order to not give any discomfort to the people in the car.

[image1]: ./images/1.png
[image2]: ./images/2.png
[image3]: ./images/3.png

![Screenshot 1][image1]

![Screenshot 2][image2]

### Path planning

The path is planned according to the approach described in the Udacity walkthrough ( https://youtu.be/7sI3VHFPP0w )

Splines have been used to plan a smooth path (i.e. minimum jerk), instead of using the jerk minimizing trajectory as explained in the lessons.

### Behaviour planning

In order to understand if the car can safely change lane, three parameters are used:

* total room available in the target lane = distance between the closest car ahead of our car and the closest car behind our car
* distance to the closest car ahead in the target lane
* distance to the closest car behind in the target lane

The car will:

* keep the current lane if there are no cars ahead and cruise at about 50 MPH
* keep the current lane and adapt its speed to the slower car ahead if it is not possible to safely change lane
* change lane if there is a slower car ahead and there is sufficient room for a safe overtake

In case two free lanes are available, the one with the bigger available room for manuever is chosen


### Prediction

Since other cars are moving at different speeds, we need to predict their positions at the end of our desidered manuever. Taking into account that a manuever (lane change) takes about 1.5 s, the sensor fusion structure is processed and future *d* and *s* values are predicted.

The predicted *d* and *s* values are used to calculate the available room on each lane.


## Cost functions

If car is in lane 0:
* if there is sufficient room for a safe manuever, cost for lane 1 is set to 0
* cost for lane 2 is set to a high value, in order to prevent unsafe manuevers

If car is in lane 1:
* cost for lane 0 is 1 / (room available ahead on lane 0 + room available behind on lane 0)
* cost for lane 2 is 1 / (room available ahead on lane 2 + room available behind on lane 2)
* lower costs correspond to more room available for a safe manuever
* in case a safe manuever is not possible, the cost for the current lane is set to 0, so that the car will keep the lane and follow the car ahead

If car is in lane 2:
* if there is sufficient room for a safe manuever, cost for lane 1 is set to 0
* cost for lane 0 is set to a high value, in order to prevent unsafe manuevers

## Comments and improvements

* the implementation of the project took much more than expected, mostly due to the difficulty of putting the concept learned in the classes together and understand how they fit the C program
* the general structure of the program should be improved, for example separating the main functions in different classes (path planning, behavior planning, cost functions, etc)
* the program should have a flexible way to add cost functions, without touching the main program
   
### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

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