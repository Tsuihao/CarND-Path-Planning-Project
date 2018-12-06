#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "utils.h"
#include "json.hpp"
#include "config.h"
#include "spline.h"
#define verbose true

using namespace std;

// for convenience
using json = nlohmann::json;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
        
            size_t prev_path_size = previous_path_x.size();
            
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
            

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
            
          	json msgJson;
            
            /* Algorithm */
            vector<double> ptsx, ptsy;
            
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);
            
            // If the prev_path is nearly empty, use the ego's current position as reference
            if(prev_path_size < 2)
            {
                // conduct the previous coordinate
                double prev_car_x = car_x - cos(ref_yaw);
                double prev_car_y = car_y - sin(ref_yaw);
                
                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);
                
                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            }
            else // you have many points to use
            {
                ref_x = previous_path_x[prev_path_size - 1]; // last
                ref_y = previous_path_y[prev_path_size - 1];
                
                double prev_x = previous_path_x[prev_path_size - 2]; // second last
                double prev_y = previous_path_y[prev_path_size - 2];
                
                ref_yaw = atan2(ref_y - prev_y, ref_x - prev_x);
                ptsx.push_back(prev_x);
                ptsx.push_back(ref_x);
                
                ptsy.push_back(prev_y);
                ptsy.push_back(ref_y);
            }
            
            // In frenet coordinate, add evenly spaced point
            // Just give some points, the rest is handled by spline.h fitting
            vector<double> next_wp_0 = getXY(car_s + 30, 2+4*config::lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp_1 = getXY(car_s + 60, 2+4*config::lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp_2 = getXY(car_s + 90, 2+4*config::lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            
            ptsx.push_back(next_wp_0[0]);
            ptsx.push_back(next_wp_1[0]);
            ptsx.push_back(next_wp_2[0]);
            
            ptsy.push_back(next_wp_0[1]);
            ptsy.push_back(next_wp_1[1]);
            ptsy.push_back(next_wp_2[1]);
            
            // Transform all the poinst as last reference point is orgin (VCS)
            for(int i = 0; i < ptsx.size(); ++i)
            {
                double shift_x = ptsx[i] - ref_x;
                double shift_y = ptsy[i] - ref_y;
                
                ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
                ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }
                
            // Spline fitting
            tk::spline spline;
            spline.set_points(ptsx, ptsy);  // set (x,y) to fitting
            
            vector<double> next_x_vals;
            vector<double> next_y_vals;
            
            // copy the previous path points
            for(int i = 0; i < previous_path_x.size(); ++i)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            
            // break up the spline points so that we can travel at the desired reference velocity
            double target_x = 30;
            double target_y = spline(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);
            
            double x_offset = 0;
            
            // fill the rest of our path planner, and we want in total 50 points
            for(int i = 1; i <= 50 - previous_path_x.size(); ++i)
            {
                // calculate how many points can maintain the desired velocity
                double N = (target_dist/(0.02* config::ref_vel/2.24));  // mph -> meter-per-second
                double x_point = x_offset + (target_x)/N;
                double y_point = spline(x_point);
                
                x_offset = x_point; // update the offset
                
                // transform back to LCS
                x_point = x_point * cos(ref_yaw) - y_point * sin(ref_yaw);
                y_point = x_point * sin(ref_yaw) + y_point * cos(ref_yaw);
                
                x_point += ref_x;
                y_point += ref_y;
                
                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
                
            }
            
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
