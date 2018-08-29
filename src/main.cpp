#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

// Parameters for the path planning
#define NUM_LANES 3
#define WIDTH_LANES 4				// m
#define UPDATE_RATE 0.02		// s
#define SAFETY_DISTANCE 30	// m
#define MAX_SPEED 22.1 			// m/s
#define MAX_ACC 0.1		 			// ms
#define HORIZON_INC 30			// m
#define HORIZON_STEPS 3
#define PLANNING_LENGTH 50

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

// Determine vehicle lane based on the d value of frenet coordinates
int determine_lane(double d) {
	for(int i = 0; i < NUM_LANES; i++) {
		if(d >= WIDTH_LANES * i && d < WIDTH_LANES * (i + 1)) {
			return i;
		}
	}

	return -1;
}

// Transform map coordinates to ego vehicle coordinates
vector<double> transform_map_to_ego(double ego_x, double ego_y, double ego_yaw, double x, double y) {
	double shift_x = x - ego_x;
	double shift_y = y - ego_y;
	double wp_x = shift_x * cos(0 - ego_yaw) - shift_y * sin(0 - ego_yaw); 
	double wp_y = shift_x * sin(0 - ego_yaw) + shift_y * cos(0 - ego_yaw);

	return {wp_x, wp_y};
}

// Transform ego vehicle coordinates to map coordinates
vector<double> transform_ego_to_map(double ego_x, double ego_y, double ego_yaw, double x, double y) {
	double shift_x = x;
	double shift_y = y;
	double wp_x = shift_x * cos(ego_yaw) - shift_y * sin(ego_yaw);
	double wp_y = shift_x * sin(ego_yaw) + shift_y * cos(ego_yaw);

	wp_x += ego_x;
	wp_y += ego_y;

	return {wp_x, wp_y};
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

	// Lane where the vehicle starts in
	int ego_lane = 1;

	// Reference velocity for the beginningto avoid abrupt start
	double reference_velocity = 0.0;

  h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy, &ego_lane, &reference_velocity](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        		// Main car's localization Data
						// The car's position in map coordinats
          	double ego_x = j[1]["x"];
          	double ego_y = j[1]["y"];
						// The car's position in frenet coordinates
          	double ego_s = j[1]["s"];
          	double ego_d = j[1]["d"];
						// The car's yaw angle in the map
          	double ego_yaw = j[1]["yaw"];
						// The car's speed in MPH
          	double ego_speed = j[1]["speed"];


          	// Previous path data given to the Planner
						// Note: Return the previous list but with processed points removed, can be
						// a nice tool to show how far along the path has processed since last time.
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
						// A 2D vector of cars and then that car's unique ID, map coordinates, velocities
						// in x and y direction, and position in frenet coordinates.
						// The data has the format: [id, x, y, vx, vy, s, d]
          	auto sensor_fusion = j[1]["sensor_fusion"];

						// Number of undriven points of the previous path
						int path_size = previous_path_x.size();

						// ENVIRONMENT ANALYSIS
						// Variables to store state of lanes around the ego vehicle to determine whether lane change is
						// possible
						bool ahead_blocked = false;
						bool left_blocked = false | ego_lane == 0;
						bool right_blocked = false | ego_lane == NUM_LANES - 1;
						// printf("left: %d, ahead: %d, right: %d\n", left_blocked, ahead_blocked, right_blocked);

						// Set ego_s to last point of previous path
						if (path_size > 0) {
							ego_s = end_path_s;
						}

						// Loop over sensor fusion data containing vehicles in the environment
						for(int i = 0; i < sensor_fusion.size(); i++) {
							// Get sensor fusion data for detected vehicle
							double veh_vx = sensor_fusion[i][3];
							double veh_vy = sensor_fusion[i][4];
							double veh_s = sensor_fusion[i][5];
							double veh_d = sensor_fusion[i][6];

							// Calculate speed of the vehicle in driving direction
							double veh_speed = sqrt(veh_vx * veh_vx + veh_vy * veh_vy);

							// Calculate vehicle lane based on d value of frenet coordinates
							int veh_lane = determine_lane(veh_d);

							// Predict the vehicle's future s position to the timepoint where the previously
							// planned trajectory ends. Assumes constant speed for that amount of timesteps.
							double veh_s_pred = veh_s + (double) path_size * veh_speed * UPDATE_RATE;
							// double ego_s_pred = ego_s + (double) path_size * ego_speed * UPDATE_RATE;

							if(ego_lane == veh_lane) {
								// Vehicle ahead in ego lane and distance
								ahead_blocked |= veh_s_pred > ego_s && veh_s_pred - ego_s < SAFETY_DISTANCE; // TODO: Check whether comparison between predictions works better
							} else if(veh_lane - ego_lane == -1 && !left_blocked) {
								// Vehicle will be in the range between -30 to 30 meters from ego vehicle
								left_blocked |= ego_s - SAFETY_DISTANCE < veh_s_pred && ego_s + SAFETY_DISTANCE > veh_s_pred; // TODO: Check whether prediction of ego works better
							} else if(veh_lane - ego_lane == 1 && !right_blocked) {
								// Vehicle will be in the range between -30 to 30 meters from ego vehicle
								right_blocked |= ego_s - SAFETY_DISTANCE < veh_s_pred && ego_s + SAFETY_DISTANCE > veh_s_pred; // TODO: Check whether prediction of ego works better
							}

						}
						// printf("left: %d, ahead: %d, right: %d\n", left_blocked, ahead_blocked, right_blocked);

						// BEHAVIOR PLANNING
						// Reduce speed or change lane
						if(ahead_blocked && !left_blocked) {
							// Change to left lane
							ego_lane--;
						} else if(ahead_blocked && !right_blocked) {
							// Change to right lane
							ego_lane++;
						} else if(ahead_blocked && right_blocked && left_blocked) {
							// If all lanes are blocked, reduce speed
							reference_velocity -= MAX_ACC;
						} else {
							// If ego lane is free accelerate up to max speed
							if(reference_velocity < MAX_SPEED) {
								reference_velocity += MAX_ACC;
							}
						}

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

						// TRAJECTORY GENERATION
						// Using previous waypoints
						double pos_x;
						double pos_y;
						double angle;

						// Put the remaining previous path in the next path for smoothing
						for(int i = 0; i < path_size; i++)
						{
								next_x_vals.push_back(previous_path_x[i]);
								next_y_vals.push_back(previous_path_y[i]);
						}

						// List of waypoints that will be interpolated
						vector<double> pts_x;
						vector<double> pts_y;

						// If there is almost nothing left from the previous path then start from scratch
						if(path_size < 2) {
								// Take the current ego vehicle information
								pos_x = ego_x;
								pos_y = ego_y;
								angle = deg2rad(ego_yaw);

								// Create a "previous" point to create a tangent to the ego vehicle
								double pos_x2 = ego_x - cos(ego_yaw);
								double pos_y2 = ego_y - sin(ego_yaw);

								// Transform to ego coordinates
								vector<double> wp_1 = transform_map_to_ego(pos_x, pos_y, angle, pos_x2, pos_y2);
								vector<double> wp_2 = transform_map_to_ego(pos_x, pos_y, angle, pos_x, pos_y);

								pts_x.push_back(wp_1[0]);
								pts_x.push_back(wp_2[0]);

								pts_y.push_back(wp_1[1]);
								pts_y.push_back(wp_2[1]);

								// pts_x.push_back(pos_x2);
								// pts_x.push_back(pos_x);

								// pts_y.push_back(pos_y2);
								// pts_y.push_back(pos_y);
						} else {
							// Coordinates of the end of the previous path
							pos_x = previous_path_x[path_size-1];
							pos_y = previous_path_y[path_size-1];

							// Coordinates of the state before the end of the path to create tangent to the ego vehicle
							double pos_x2 = previous_path_x[path_size-2];
							double pos_y2 = previous_path_y[path_size-2];
							angle = atan2(pos_y - pos_y2, pos_x - pos_x2);

							// Transform to ego coordinates
							vector<double> wp_1 = transform_map_to_ego(pos_x, pos_y, angle, pos_x2, pos_y2);
							vector<double> wp_2 = transform_map_to_ego(pos_x, pos_y, angle, pos_x, pos_y);

							pts_x.push_back(wp_1[0]);
							pts_x.push_back(wp_2[0]);

							pts_y.push_back(wp_1[1]);
							pts_y.push_back(wp_2[1]);

							// pts_x.push_back(pos_x2);
							// pts_x.push_back(pos_x);

							// pts_y.push_back(pos_y2);
							// pts_y.push_back(pos_y);
						}

						// Setting future waypoints over a certain planning horizon
						for(int i = 1; i <= HORIZON_STEPS; i++) {
							vector<double> next_wp = getXY(ego_s + i * HORIZON_INC, (double) WIDTH_LANES * (double) ego_lane + (double) WIDTH_LANES / 2.0, map_waypoints_s, map_waypoints_x, map_waypoints_y);

							// Transform waypoint coordinates to ego vehicle coordinates
							vector<double> wp = transform_map_to_ego(pos_x, pos_y, angle, next_wp[0], next_wp[1]);

							pts_x.push_back(wp[0]);
							pts_y.push_back(wp[1]);
						}

						// printf("PTS_X\n");
						// printf("pts_x[0]: %f\n", pts_x[0]);
						// printf("pts_x[1]: %f\n", pts_x[1]);
						// printf("pts_x[2]: %f\n", pts_x[2]);
						// printf("pts_x[3]: %f\n", pts_x[3]);
						// printf("pts_x[5]: %f\n", pts_x[4]);

						// Use spline package to fit a curve to the points
						tk::spline spl;
						spl.set_points(pts_x, pts_y);

						// Calculate the distance from ego vehicle to intermediate goal at planning horizon
						double goal_x = PLANNING_LENGTH * UPDATE_RATE * reference_velocity;
						double goal_y = spl(goal_x);

						double distance = sqrt(goal_x * goal_x + goal_y * goal_y);

						double x_offset = 0;

						// Fill the next_vals to planning length
						for(int i = 0; i < PLANNING_LENGTH - path_size; i++) {
							double x = x_offset + reference_velocity * UPDATE_RATE;
							double y = spl(x);

							x_offset = x;

							// Transform back to map coordinate system
							vector<double> wp = transform_ego_to_map(pos_x, pos_y, angle, x, y);

							next_x_vals.push_back(wp[0]);
							next_y_vals.push_back(wp[1]); 
						}

						// printf("NEXT_VALS\n");
						// printf("next_x_vals[0]: %f\n", next_x_vals[0]);
						// printf("next_x_vals[1]: %f\n", next_x_vals[1]);
						// printf("next_x_vals[2]: %f\n", next_x_vals[2]);
						// printf("next_x_vals[3]: %f\n", next_x_vals[3]);
						// printf("next_x_vals[5]: %f\n", next_x_vals[4]);

          	json msgJson;

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
