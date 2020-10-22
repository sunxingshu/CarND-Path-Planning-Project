#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "json.hpp"
#include "spline.h"
#include "helpers.h"

using namespace std;

// for convenience
using json = nlohmann::json;


// for convenience
using nlohmann::json;
using std::string;
using std::vector;


const double MAX_VEL = 49.5;
const double MAX_ACC = .448/2;
const int LEFT_LANE = 0;
const int MIDDLE_LANE = 1;
const int RIGHT_LANE = 2;
const int INVALID_LANE = -1;
const int LEFT_LANE_MAX = 4;
const int MIDDLE_LANE_MAX = 8;
const int RIGHT_LANE_MAX = 12;
const int PROJECTION_IN_METERS = 30;



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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  //Xingshu
  // start in lane 1
  int lane = 1;
  
  // reference velocity
  double ref_vel = 0.0; //mph

  bool car_ahead = false;
  bool car_left = false;
  bool car_right = false;

  // Initialize FSM
  fsm.add_transitions({
                            //  from state ,to state  ,triggers        ,guard                    ,action
                            { States::Normal  ,States::ChangeLeft ,Triggers::CarAhead  ,[&]{return car_ahead && !car_left && lane > LEFT_LANE;}  ,[&]{lane--;} },
                            { States::ChangeLeft ,States::Normal ,Triggers::Clear  ,[&]{return !car_ahead;}  ,[&]{} },
                            { States::ChangeLeft ,States::Follow ,Triggers::CarAhead  ,[&]{return car_ahead;}  ,[&]{ ref_vel -= MAX_ACC; } },
                            { States::Follow ,States::ChangeLeft ,Triggers::CarAhead  ,[&]{return car_ahead && !car_left && lane > LEFT_LANE;}  ,[&]{lane--;} },

                            { States::Normal  ,States::ChangeRight ,Triggers::CarAhead  ,[&]{return car_ahead && !car_right && lane != RIGHT_LANE;}  ,[&]{lane++;} },
                            { States::ChangeRight  ,States::Normal ,Triggers::Clear  ,[&]{return !car_ahead;}  ,[&]{} },
                            { States::ChangeRight ,States::Follow ,Triggers::CarAhead  ,[&]{return car_ahead;}  ,[&]{ ref_vel -= MAX_ACC; } },
                            { States::Follow ,States::ChangeRight ,Triggers::CarAhead  ,[&]{return car_ahead && !car_right && lane != RIGHT_LANE;}  ,[&]{lane++;} },

                            { States::Normal  ,States::Follow ,Triggers::CarAhead  ,[&]{return true;}  ,[&]{ref_vel -= MAX_ACC;} },
                            { States::Follow  ,States::Follow ,Triggers::CarAhead  ,[&]{return true;}  ,[&]{ref_vel -= MAX_ACC;} },
                            { States::Follow  ,States::Normal ,Triggers::Clear  ,[&]{return !car_ahead;}  ,[&]{ref_vel += MAX_ACC;} },
                            { States::Normal  ,States::Normal ,Triggers::Clear  ,[&]{return !car_ahead;}  ,[&]{ if (ref_vel < MAX_VEL) { ref_vel += MAX_ACC; }} },

                    });

  fsm.add_debug_fn(dbg_fsm);

  h.onMessage([&car_ahead, &car_left, &car_right, &ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
              car_s = end_path_s;
          }

          // 1. PREDICTION : Analysing other cars positions.
          car_ahead = false;
          car_left = false;
          car_right = false;    
              
          for ( int i = 0; i < sensor_fusion.size(); i++ ) {

              float d = sensor_fusion[i][6];
              int other_car_lane = INVALID_LANE;

              // Determine the lane of the other car
              if ( d > 0 && d < LEFT_LANE_MAX ) {
                  other_car_lane = LEFT_LANE;
              } else if ( d > LEFT_LANE_MAX && d < MIDDLE_LANE_MAX ) {
                  other_car_lane = MIDDLE_LANE;
              } else if ( d > MIDDLE_LANE_MAX && d < RIGHT_LANE_MAX ) {
                  other_car_lane = RIGHT_LANE;
              }
              if (other_car_lane == INVALID_LANE) {
                  continue;
              }

              // Determine the speed of the other car
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx + vy*vy);
              double check_car_s = sensor_fusion[i][5];

              // Estimate the other cars' position after executing previous trajectory
              check_car_s += (double) prev_size * 0.02 * check_speed;

              if ( other_car_lane == lane ) {
                  // Other car is in the same lane
                  car_ahead |= (check_car_s > car_s) && (check_car_s - car_s < PROJECTION_IN_METERS);
              } else if ( other_car_lane - lane == -1 ) {
                  // Other car is on the left lane
                  car_left |= (car_s - PROJECTION_IN_METERS < check_car_s) && (car_s + PROJECTION_IN_METERS > check_car_s);
              } else if ( other_car_lane - lane == 1 ) {
                  // Other car is on the right lane
                  car_right |= (car_s - PROJECTION_IN_METERS < check_car_s) && (car_s + PROJECTION_IN_METERS > check_car_s);
              }
          }

            // 2. BEHAVIOR: Trigger State Changes Depending If Road Clear of Vehicle Ahead
            if (car_ahead) {
                // Execute 'CarAhead' trigger on state machine
                fsm.execute(Triggers::CarAhead);
            } else {
                // Execute 'Clear' trigger on state machine
                fsm.execute(Triggers::Clear);
            }


            // 3. TRAJECTORY: Calculate trajectory for the car to follow

            // Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
            // later interpolate waypoints with spline and fill in more waypoints

            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            // If previous size is almost empty, use car as starting reference
            if (prev_size < 2) {
                // Use two points that make the path tangent to the car
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }
            // Use previous path's points as starting reference
            else {
                // Redefine reference state as previous path endpoint
                ref_x = previous_path_x[prev_size-1];
                ref_y = previous_path_y[prev_size-1];

                double ref_x_prev = previous_path_x[prev_size-2];
                double ref_y_prev = previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

                // Use two points that make the path tangent to the previous path's endpoint
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }

            // In Frenet add evenly 30m spaced points ahead of the starting reference
            vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            for (int i=0; i<ptsx.size(); i++) {
                // Shift car angle reference to 0 degrees
                double shift_x = ptsx[i]-ref_x;
                double shift_y = ptsy[i]-ref_y;

                ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
                ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }

            // Create a spline
            tk::spline s;

            // Set (x,y) points to the spline
            s.set_points(ptsx, ptsy);

            // Define the actual (x,y) points that will be used for the planner
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // Start with all of the previous path points from last time
            for (int i=0; i < previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // Calculate how to break up spline points so that the desired refrence velocity is kept
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

            double x_add_on = 0;

            // Fill up the rest of the path planner after filling it with previous points
            // Always 50 points will be output
            for (int i=0; i <= 50-previous_path_x.size(); i++) {

                double N = (target_dist/(0.02*ref_vel/2.24));
                double x_point = x_add_on+(target_x)/N;
                double y_point = s(x_point);

                x_add_on = x_point;

                double x_ref = x_point;
                double y_ref = y_point;

                //Rotate back to normal after rotating earlier
                x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
                y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

                x_point += ref_x;
                y_point += ref_y;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
            }
          
          json msgJson;

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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