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
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

class AutonomousVehicle{

private:
  int ego_lane;
  double ego_x;
  double ego_y;
  double ego_s; 
  double ego_d; 
  double ego_yaw;
  double ego_speed;
  double SAFETY_DISTANCE; //meters
  double ref_v; 
  double SPEED_LIMIT;// 49.5mph = 22.098m/s
  double ego_future_s;
  string ego_state; // state includes KL - Keep Lane / LCL - Lane Change Left / LCR - Lane Change Right
  vector<double> lane_speed;
  vector<double> lane_frontcar_s;
  vector<double> lane_backcar_s;

public:

  AutonomousVehicle(){
    ego_lane = 1; 
    SAFETY_DISTANCE = 25.0;
    ref_v = 0.0;
    SPEED_LIMIT = 49.5;
    lane_speed = {SPEED_LIMIT, SPEED_LIMIT, SPEED_LIMIT};
    lane_frontcar_s = {numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};
    lane_backcar_s = {numeric_limits<double>::min(), numeric_limits<double>::min(), numeric_limits<double>::min()};
    ego_state = "KL";
    cout << "Autonomous Vehicle created at lane 1" << endl;
  }
  virtual ~AutonomousVehicle() {}

  //function to update ego with simulator returned localized data
  void update_position(double x, double y, double s, double d, double yaw, double speed) {
    ego_x = x;
    ego_y = y;
    ego_s = s;
    ego_d = d;
    ego_yaw = yaw;
    ego_speed=speed;
  };

  //function to update state choice based on traffice situation
  void update_state(const vector<double> &previous_path_x, const double &end_path_s, const vector<vector<double>> &sensor_fusion) {
    
    int prev_size = previous_path_x.size();

    //variables for cars being checked
    double vx;
    double vy;
    double check_speed;
    double check_car_s;
    double check_car_future_s;

    //if previous planning was done, set ego_future_s to last known end_path_s point
    if(prev_size > 0){
      ego_future_s = end_path_s;
    }
    //else set ego_future_s to current ego_s position
    else{
      ego_future_s = ego_s;
    }

    //Reset lane_speed to max road speed limit
    lane_speed = {SPEED_LIMIT, SPEED_LIMIT, SPEED_LIMIT};
    //Reset lane_frontcar_s to max double
    lane_frontcar_s = {numeric_limits<double>::max(), numeric_limits<double>::max(), numeric_limits<double>::max()};
    //Reset lane_backcar_s to min double
    lane_backcar_s = {numeric_limits<double>::min(), numeric_limits<double>::min(), numeric_limits<double>::min()};

    /***********************
    Step 1
    Iterate through all vehicles in all lanes and find vehicle that is closest (infront and behind) to ego 
    to determine lane speed
    
    * Information recorded for all 3 lanes include lane_speed, lane_frontcar_s and lane_backcar_s.
    * Data format for each car is: [id, x, y, vx, vy, s, d].
    *************************/
    for(int i = 0; i < sensor_fusion.size(); i++)
    {
      float d = sensor_fusion[i][6];

      //check lane 0
      if(d < 4 && d > 0){
        vx = sensor_fusion[i][3];
        vy = sensor_fusion[i][4];
        check_speed = sqrt(vx*vx+vy*vy);
        check_car_s = sensor_fusion[i][5];

        check_car_future_s = check_car_s + ((double)prev_size*.02*check_speed);

        //if vehicle is infront of ego and lesser than 50m at final projected position record 
        //target vehicle s location and speed. Only vehicle infront and closest to ego is recorded
        if((check_car_future_s > ego_future_s) && (abs(check_car_future_s-ego_future_s) < 50.0)){

          if(abs(check_car_future_s-ego_future_s) < abs(lane_frontcar_s[0]-ego_future_s)){
            lane_speed[0] = check_speed*2.237;
            lane_frontcar_s[0] = check_car_future_s;
          }

        }
        //if vehicle is behind ego and lesser than SAFETY_DISTANCE at final projected position record 
        //target vehicle s location. Only vehicle behind and closest to ego is recorded.
        else if((check_car_future_s < ego_future_s) && (abs(check_car_future_s-ego_future_s) < SAFETY_DISTANCE)){
          
          if(abs(check_car_future_s-ego_future_s) < abs(lane_backcar_s[0]-ego_future_s)){

            lane_backcar_s[0] = check_car_future_s;

          }
        }

      }
      //check lane 1
      else if(d < 8 && d > 4){
        vx = sensor_fusion[i][3];
        vy = sensor_fusion[i][4];
        check_speed = sqrt(vx*vx+vy*vy);
        check_car_s = sensor_fusion[i][5];

        check_car_future_s = check_car_s + ((double)prev_size*.02*check_speed);

        //if vehicle is infront of ego and lesser than 50m at final projected position record 
        //target vehicle s location and speed. Only vehicle infront and closest to ego is recorded
        if((check_car_future_s > ego_future_s) && ((check_car_future_s-ego_future_s) < 50.0)){
          if(abs(check_car_future_s-ego_future_s) < abs(lane_frontcar_s[1]-ego_future_s)){
            lane_speed[1] = check_speed*2.237;
            lane_frontcar_s[1] = check_car_future_s;
          }
        }
        //if vehicle is behind ego and lesser than SAFETY_DISTANCE at final projected position record 
        //target vehicle s location. Only vehicle behind and closest to ego is recorded.
        else if((check_car_future_s < ego_future_s) && (abs(check_car_future_s-ego_future_s) < SAFETY_DISTANCE)){
          if(abs(check_car_future_s-ego_future_s) < abs(lane_backcar_s[1]-ego_future_s)){

            lane_backcar_s[1] = check_car_future_s;
            
          }
        }
      }
      //check lane 2
      else if(d < 12 && d > 8){
        vx = sensor_fusion[i][3];
        vy = sensor_fusion[i][4];
        check_speed = sqrt(vx*vx+vy*vy);
        check_car_s = sensor_fusion[i][5];

        check_car_future_s = check_car_s + ((double)prev_size*.02*check_speed);

        //if vehicle is infront of ego and lesser than 50m at final projected position record 
        //target vehicle s location and speed. Only vehicle infront and closest to ego is recorded
        if((check_car_future_s > ego_future_s) && ((check_car_future_s-ego_future_s) < 50.0)){
          if(abs(check_car_future_s-ego_future_s) < abs(lane_frontcar_s[2]-ego_future_s)){
            lane_speed[2] = check_speed*2.237;
            lane_frontcar_s[2] = check_car_future_s;
          }
        }
        //if vehicle is behind ego and lesser than SAFETY_DISTANCE at final projected position record 
        //target vehicle s location. Only vehicle behind and closest to ego is recorded.
        else if((check_car_future_s < ego_future_s) && (abs(check_car_future_s-ego_future_s) < SAFETY_DISTANCE)){
          if(abs(check_car_future_s-ego_future_s) < abs(lane_backcar_s[2]-ego_future_s)){

            lane_backcar_s[2] = check_car_future_s;
            
          }
        }
      }
    }

    /***********************
    Step 2
    Use previously found closest car position and speed in all 3 lanes to determine ego's preferred state
    ego's aim is to complete the circuit safely and in the shortest possible time
    *************************/

    //ideal_lane is the lane that allows highest travel speed
    int ideal_lane = distance(lane_speed.begin(), max_element(lane_speed.begin(), lane_speed.end()));

    //if ego is already travelling at max speed limit of the road, keep in current lane
    if(lane_speed[ego_lane] == SPEED_LIMIT){
      ego_state = "KL";
    }
    else{

      if(ego_lane == 0){
        //if ego is in lane 0 and is also the ideal_lane, keep lane
        if(ego_lane == ideal_lane){
          ego_state = "KL";
        }
        //else Lane Change Right
        else{
          ego_state = "LCR";
        }
        
      }
      else if(ego_lane == 1){

        //if ego is in lane 1 and is also the ideal_lane, keep lane
        if(ego_lane == ideal_lane){
          ego_state = "KL";
        }
        else if(ego_lane > ideal_lane){

          //else if ideal lane is to the left, check if right lane is also at max road speed of 49.5 and no vehicle behind
          //this is due to perculiarity of both lane 0 and 2 are at max road speed of 49.5 and preference should be
          //given to the one with no vehicle behind that might prevent lane changes
          if(lane_speed[ego_lane+1] == 49.5 && lane_backcar_s[ego_lane+1] == numeric_limits<double>::min()){
            ego_state = "LCR";
          }
          else{
            ego_state = "LCL";
          }
          
        }
        else{
          //if ideal_lane is on lane 2, perform Lane Change Right
          ego_state = "LCR";
        }
      }
      else if(ego_lane == 2){

        //if ego is in lane 2 and is also the ideal_lane, keep lane
        if(ego_lane == ideal_lane){
          ego_state = "KL";
        }
        //else Lane Change Left
        else{
          ego_state = "LCL";
        }
      }
    }
  };

  vector<vector<double>> realize_state(const vector<double> &previous_path_x, const vector<double> &previous_path_y, 
                    const vector<double> &map_waypoints_x, const vector<double> &map_waypoints_y, 
                    const vector<double> &map_waypoints_s) {
    /***********************
    Based on individual state and the objectives of the state, adjust ref_v so as to achieve the state safely. ego_lane
    variable will also be changed once lane change is determined to be safe for execution.
    *************************/

    vector<double> next_x_vals;
    vector<double> next_y_vals;
    int prev_size = previous_path_x.size();

    cout << "lane 0: " << lane_speed[0] << " lane 1: " << lane_speed[1] << " lane 2: " << lane_speed[2] << endl;
    cout << "Reference Velocity: " << ref_v << " Lane Velocity: " << lane_speed[ego_lane] << endl;            
    cout << "Selected State: " << ego_state << endl;

    if(ego_state == "KL"){

      //Code to maintain lane speed and sufficient separaton between ego and front car
      if(ref_v < SPEED_LIMIT && lane_frontcar_s[ego_lane] - ego_future_s > SAFETY_DISTANCE){

        ref_v += .224;
      }
      //else decrease speed to maintain safety distance and safe speed
      else{

        ref_v -= .224;

        //perform emergency breaking if front car future s and ego_future_s separation is less than 10m
        if(lane_frontcar_s[ego_lane] - ego_future_s < 10){
          ref_v -= 2.0;
        }

      }

    }
    else if(ego_state == "LCL"){

      //Code to maintain lane speed and sufficient separaton between ego and front car
      if(ref_v < SPEED_LIMIT && lane_frontcar_s[ego_lane] - ego_future_s > SAFETY_DISTANCE){

        //if Lane Change Left and velocity is lesser than current lane speed, check target lane speed and reduce speed 
        //to 5MPH slower than target lane speed to find opportunity to change lane
        if(ref_v < lane_speed[ego_lane-1] - 5.0){

          ref_v += .224;
        }
        else{
          ref_v -= .224;
        }
      }
      //else decrease speed to maintain safety distance and safe speed
      else{

        ref_v -= .224;

        //perform emergency breaking if front car future s and ego_future_s separation is less than 10m
        if(lane_frontcar_s[ego_lane] - ego_future_s < 10){
          ref_v -= 2.0;
        }
      }

      //check target lane and accelerate during lane change if safe to change lane
      //maintaince 30m from front car and 20m from back car
      if(lane_frontcar_s[ego_lane-1]-ego_future_s > SAFETY_DISTANCE){

        if(ego_future_s-lane_backcar_s[ego_lane-1] > SAFETY_DISTANCE-10.0){
          
          ego_lane = ego_lane - 1;

        }
        
      }

    }
    else if(ego_state == "LCR"){

      if(ref_v < SPEED_LIMIT && lane_frontcar_s[ego_lane] - ego_future_s > SAFETY_DISTANCE){

        if(ref_v < lane_speed[ego_lane+1]-5.0){
          ref_v += .224;
        }
        else{
          ref_v -= .224;
        }
      }
      //else perform breaking as per emergency or normal
      else{

        ref_v -= .224;
        //perform emergency breaking if front car future s and ego_future_s separation is less than 10
        if(lane_frontcar_s[ego_lane] - ego_future_s < 10){
          ref_v -= 2.0;
        }
      }

      //check target lane and accelerate during lane change if safe to change lane
      //maintaince 30m from front car and 20m from back car
      if(lane_frontcar_s[ego_lane+1]-ego_future_s > SAFETY_DISTANCE){

        if(ego_future_s-lane_backcar_s[ego_lane+1] > SAFETY_DISTANCE-10.0){

          ego_lane = ego_lane + 1;

        }
        
      }
    }

    /***********************
    Trajectory generation using ref_v set in earlier
    *************************/

    //list of spaced (x,y) points evenly spaced at 30m
    vector<double> ptsx;
    vector<double> ptsy;

    //reference x, y and yaw states
    double ref_x = ego_x;
    double ref_y = ego_y;
    double ref_yaw = deg2rad(ego_yaw);

    //if prev path is almost empty, use the car position as starting reference
    if(prev_size < 2){

      //create previous car position in (x,y) map coordinate using the car current position
      double prev_car_x = ego_x - cos(ego_yaw);
      double prev_car_y = ego_y - sin(ego_yaw);

      //push back into ptsx and ptsy the 2 points for the tangent line
      ptsx.push_back(prev_car_x);
      ptsx.push_back(ego_x);

      ptsy.push_back(prev_car_y);
      ptsy.push_back(ego_y);
    }
    //if prev path is not empty, use previous path endpoint at starting point
    else{

      ref_x = previous_path_x[prev_size-1];
      ref_y = previous_path_y[prev_size-1];

      double ref_x_prev = previous_path_x[prev_size-2];
      double ref_y_prev = previous_path_y[prev_size-2];

      ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

      //push back into ptsx and ptsy the 2 points for the tangent line
      ptsx.push_back(ref_x_prev);
      ptsx.push_back(ref_x);

      ptsy.push_back(ref_y_prev);
      ptsy.push_back(ref_y);
    }

    //add another 3 waypoints where each waypoints are spaced 30m apart
    //convert these waypoints from Frenet back to (x,y) map space
    vector<double> next_wp0 = ::getXY(ego_future_s+30,(2+4*ego_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
    vector<double> next_wp1 = ::getXY(ego_future_s+60,(2+4*ego_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
    vector<double> next_wp2 = ::getXY(ego_future_s+90,(2+4*ego_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    for (int i = 0; i < ptsx.size(); i++ )
    {
      //use every previous ptsx and ptsy values (waypoints x,y values)
      //find the relative position of these points to car position
      //and convert them from map space to car space (car reference frame)
      //car is now at (0,0) origin
      double shift_x = ptsx[i]-ref_x;
      double shift_y = ptsy[i]-ref_y;

      //changing from map space to car space by using rotation matrix for each waypoints
      //https://en.wikipedia.org/wiki/Rotation_matrix
      ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
      ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

    }
    
    //create a spline
    tk::spline s;

    //fit all previous waypoints to the spline
    //waypoints are now in car reference frame
    s.set_points(ptsx,ptsy);

    //push all previous left over untravelled paths into next_x_vals and next_y_vals
    //for continuity in path planning
    for(int i = 0; i < previous_path_x.size(); i++)
    {
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

    //assume a target_x 30m away
    //fill up additional points within this 30m distance using spline provided y values
    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y)); //assume straight line

    double x_add_on = 0;

    //top up the rest of the planner next_x_vals and next_y_vals with new planned points
    for (int i = 1; i <= 50-previous_path_x.size(); i++) {

      //find new x_point and y_point
      double N = (target_dist/(.02*ref_v/2.24));
      double x_point = x_add_on+(target_x)/N;
      double y_point = s(x_point);

      //update x_add_on to the new point
      x_add_on = x_point;

      //set x_ref and y_ref to new x_point and y_point
      //x_ref and y_ref represents the next point after the car's current location
      double x_ref = x_point;
      double y_ref = y_point;

      //rotate from car space back to map space but without translation
      //i.e. still taking car as centre (0,o)
      x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
      y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

      //convert x_point and y_point with translation to account for car's position on map
      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);

    }

    return {next_x_vals,next_y_vals};
  }
};

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

  
  /***********************
  Define a new AutonomousVehicle instance named ego
  *************************/

  AutonomousVehicle ego;

  h.onMessage([&ego, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

            // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

            //Update ego with its latest position from simulator
            ego.update_position(car_x, car_y, car_s, car_d, car_yaw, car_speed);

            //ego has 3 states namely Keep Lane (KL), Lane Change Left (LCL) and Lane Change Right(LCR)
            //update its current preferred state based on current road situation
            ego.update_state(previous_path_x, end_path_s, sensor_fusion);
            
            //realize the chosen state by executing the state once it is safe to do so.
            vector<vector<double>> path = ego.realize_state(previous_path_x, previous_path_y, map_waypoints_x, map_waypoints_y, map_waypoints_s);

          	msgJson["next_x"] = path[0];
          	msgJson["next_y"] = path[1];

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
















































































