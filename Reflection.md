# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

![Project_Image](./images/collision_free_driving_1.png)

## Reflection

### Implementation
The implementation makes use of a third party [spline package](http://kluge.in-chemnitz.de/opensource/spline/) to smooth the trajectories of the ego vehicle. Besides, the material from the Udacity course as well as the [walkthrough/Q&A](https://www.youtube.com/watch?v=7sI3VHFPP0w) was used as a framework for how to approach this project.

### Approach

#### Preparation
For the project an iterative approach was followed, where small parts where implemented and immediately tested in the simulator. The first step was to make sure the simulator and the path planning code are communicating with each other. Therefore, a few lines of code were added that let the car follow a straight line. The code uses a constant `reference_velocity` and the fixed `update_rate` of `0.02` seconds to calculate the distance between waypoints.

```c++
// STRAIGHT LINE

// Setting the points 0.5m apart with a rate of 0.02 seconds results in approximateley 25m/s which is approximately 50mph 

double reference_velocity = 22.1; // in m/s as 22.1 m/s ~ 50 mph
double update_rate = 0.02; // in s
double dist_inc = reference_velocity * update_rate;
for(int i = 0; i < 50; i++) {
  next_x_vals.push_back(ego_x + (dist_inc * i) * cos(deg2rad(ego_yaw)));
  next_y_vals.push_back(ego_y + (dist_inc * i) * sin(deg2rad(ego_yaw)));
}
```

Next, the code provided in the course has been used to let the ego vehicle drive a circle and see how adding on the previous path works. The code checks the size of `previous_path_x` to know whether there are waypoints from a previous calculation left that have not been executed by the car yet. If there are no previous waypoints, the ego_vehicle is taken as reference. Otherwise, the previous calculation is taken as reference. Based on the reference the `next_x_vals` and `next_y_vals` are filled with values up until a total size of 50 waypoints which covers a planning horizon of 1 second because of the update rate of 0.02 seconds. 

```c++
// CIRCLE

// Using previous waypoints
double pos_x;
double pos_y;
double angle;

// Number of undriven points of the previous path
int path_size = previous_path_x.size();

// If there is almost nothing left from the previous path then start from scratch
if(path_size == 0) {
    // Take the current ego vehicle information
    pos_x = ego_x;
    pos_y = ego_y;
    angle = deg2rad(ego_yaw);
} else {
  // Coordinates of the end of the previous path
  pos_x = previous_path_x[path_size-1];
  pos_y = previous_path_y[path_size-1];

  double pos_x2 = previous_path_x[path_size-2];
  double pos_y2 = previous_path_y[path_size-2];
  angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
}

// Distance between two path points set to 0.5m which together with a frame 
// rate of 0.02s results in approximately 25m/s or 50 mph.
double reference_velocity = 22.1; // in m/s as 22.1 m/s ~ 50 mph
double update_rate = 0.02; // in s
double dist_inc = reference_velocity * update_rate;
// Fill remaining path points to have 50 path points in total
for(int i = 0; i < 50 - path_size; i++)
{    
  pos_x += (dist_inc) * cos(angle + (i + 1) * (pi() / 100));
  pos_y += (dist_inc) * sin(angle + (i + 1) * (pi() / 100));
  next_x_vals.push_back(pos_x);
  next_y_vals.push_back(pos_y);
}
```

Finally, a code snipped from the walkthrough was used to use Frenet coordinantes to follow the lane. It calculates the distance between two waypoints for driving at a constant speed. This `dist_inc` is added to the s position of the ego vehicle `ego_s`, which gives the `next_s` value of the waypoint. The `next_d` value is set to `6` because the car should drive in the center of the middle lane with a lane width of 4 m. Those `next_s` and `next_d` values are converted to x- and y-coordinates and appended to `next_x_vals` and `next_y_vals`.

```c++
// FOLLOW LANE

double reference_velocity = 22.1; // in m/s as 22.1 m/s ~ 50 mph
double update_rate = 0.02; // in s
double dist_inc = reference_velocity * update_rate;
for(int i = 0; i < 50 - path_size; i++) {
  double next_s = ego_s + (i + 1) * dist_inc;
  double next_d = 6;  // Middle lane
  vector<double> xy = getXY(next_s, next_d, map_waypoint_s, map_waypoints_x, map_waypoints_y);

  next_x_vals.push_back(xy[0]);
  next_y_vals.push_back(xy[1]);
}
```

Those were the baseline experiments to start the development for the path planning project. The final implementation builds on the code and the insights of this and expands the functionality. It can be separated into three main sections `Environment Analysis`, `Behavior Planning`, and `Trajectory Generation`.

#### [Trajectory Generation](./src/main.cpp#L372)

The trajectory generation code first fills the remaining (if any) waypoints of the previous path into the `next_x_vals` and `next_y_vals` vectors. Those two vectors contain the future waypoints of the ego vehicle that are sent to the simulator for execution. Then the vectors `pts_x` and `pts_y` are created that will store spaced out waypoints which later will be interpolated. The values in those two vectors act as `anchor waypoints` on which to orientate the fine grained trajectory. The first two values in those vectors will be used to form a tangent to either the position of the vehicle or the last state of the previous planning depending on the amount of remaining waypoints in the `previous_path`. Next, there will be additional anchor waypoints created that are evenly spaced out and add up to a maximum horizon of `HORIZON_STEPS` * `HORIZON_INC`. Transforming the anchor points from the global map coordinate systems to the ego vehicle coordinate system makes working with them easier. Those points are used to create a spline which helps to smooth the trajectory. Therefore, an external [spline package](http://kluge.in-chemnitz.de/opensource/spline/) is used which is imported from [spline.h](./src/spline.h).

In the next part of the trajectory generation section, the waypoints  that should be passed to the simulator are created by using the spline. Because the ego vehicle is driving on a highway where no strong curves occur, it is possible to linearize the distance from the ego vehicle to the desired goal. Then the `next_x_vals` and `next_y_vals` are filled up with the newly created values so that they contain `PLANNING_LENGTH` values in total (previos_path + newly created). Therefore, the distance increment (constant velocity * update rate) is added to the x value of the predecessor and the y value is determined using the spline. These coordinates are then transformed back to global map coordinates before they are added to `next_x_vals` and `next_y_vals` respectively.

#### [Environment Analysis](./src/main.cpp#L305)

The environment analysis code analysis the environment by looking at the sensor fusion data. The main purpose of this section is to determine whether the ego lane and the lanes left and right to it are blocked by other vehicles. Therefore three flags `ahead_blocked`, `left_blocked`, and `right_blocked` are created. Those flags are raised when there are other vehicles detected in the respective lane relative to the ego vehicle or when the ego vehicle is already in the outermost lane on the left or right side to avoid driving off the road. To analyze whether vehicles are blocking the lane of the subject vehicle is calculated over the Frenet coordinate d value given by the sensor fusion data. After that the speed of the subject vehicle is calculated in order to predict the future state of the subject vehicle. The future state is then used to analyze whether the vehicle will be within the safety area of the ego vehicle. If this is the case the respective flag will be raised depending on the lane the subject vehicle is driving in.

This is a simplified solution and for further improvement other factors can be taken into account, e.g. working with cost functions, not only analyzing the position of other vehicles but also the speed difference to the ego vehicle, and many more.

#### [Behavior Planning](./src/main.cpp#L351)

The behavior planning section decides whether the ego vehicle should change the lane, accelerate or deccelerat. Therefore, it checks the flags of the enivronment analysis `ahead_blocked`, `left_blocked`, and `right_blocked`. If the road in the ego lane is blocked and one of the neighboring lanes is free, the behavior planner will initiate a lane change. If every lane around the ego vehicle is blocked, the behavior planner will deccelerate. Finally, if the ego lane is not blocked, the behvaior planner will accelerate up until the `MAX_SPEED` limit.