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
#include "Vehicle.h"

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
//
// bool same_lane_speed_control(auto sensor_fusion, double car_s, double d, int lane, double &ref_vel){
//
//   bool too_close = false;
//   // create ref_v for future use
//   for(int i =0; i < sensor_fusion.size(); i++){
//     // car in ego car lane
//     float d = sensor_fusion[i][6];
//     cout << "d for car " << d << "car_id" << sensor_fusion[i][0] << endl;
//     if(d < (2 + 4*lane+2) && d > (2+4*lane-2)){
//       cout << "found car " << sensor_fusion[i][0] << "on lane "<< lane<< endl;
//       double vx = sensor_fusion[i][3]; // x velocity
//       double vy = sensor_fusion[i][4]; // y velocity
//       double check_speed = sqrt(vx*vx+vy*vy); // magnitude of velocity
//       double check_car_s = sensor_fusion[i][5];
//       check_car_s += ((double) (prev_size) * .02 * check_speed);
//
//       if((check_car_s > car_s) && ((check_car_s-car_s) < 30)){
//         // check the car s with car in front us. if the buffer 30 m then slow down or
//         // consider change lane.
//
//         //ref_vel = 29.5; // mph
//         too_close = true;
//
//         // consider change lane or use JMT to slow down the ego car
//       }
//     }
//   }
//
//
//   if(too_close){
//     ref_vel -= .224;
//   } else if(ref_vel < 49.5){
//     ref_vel += .224;
//   }
//
//   return too_close;
// }

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  Vehicle ego;
  ego.state = "KL";
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
            vector<double> prev_path_x;
            vector<double> prev_path_y;

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            vector<Vehicle> vehicles;
            ego.update_ego(-1, car_x, car_y, car_s, car_d, car_yaw, car_speed,
                           prev_path_x, prev_path_y, end_path_s, end_path_d,
                           map_waypoints_x, map_waypoints_y, map_waypoints_s,
                           map_waypoints_dx, map_waypoints_dy);

            for(int i= 0; i<previous_path_x.size();i++)
            {
              prev_path_x.push_back(previous_path_x[i]);
              prev_path_y.push_back(previous_path_y[i]);
            }

            // int prev_size = previous_path_x.size();
            // int lane = ego.lane;
            // cout << "ego lane " << ego.lane << endl;
            // if(prev_size > 0){
            //   car_s = end_path_s; // previous last point s
            // }
            // bool too_close = false;
            // // create ref_v for future use
            // for(int i =0; i < sensor_fusion.size(); i++){
            //   // car in ego car lane
            //   float d = sensor_fusion[i][6];
            //   cout << "d for car " << d << "car_id " << sensor_fusion[i][0] << endl;
            //   if(d < (2 + 4*lane+2) && d > (2+4*lane-2)){
            //     cout << "found car " << sensor_fusion[i][0] << "on lane "<< lane<< endl;
            //     double vx = sensor_fusion[i][3]; // x velocity
            //     double vy = sensor_fusion[i][4]; // y velocity
            //     double check_speed = sqrt(vx*vx+vy*vy); // magnitude of velocity
            //     double check_car_s = sensor_fusion[i][5];
            //     check_car_s += ((double) (prev_size) * .02 * check_speed);
            //
            //     if((check_car_s > car_s) && ((check_car_s-car_s) < 30)){
            //       // check the car s with car in front us. if the buffer 30 m then slow down or
            //       // consider change lane.
            //
            //       //ref_vel = 29.5; // mph
            //       too_close = true;
            //
            //       // consider change lane or use JMT to slow down the ego car
            //     }
            //   }
            // }
            //
            //
            // if(too_close){
            //   ego.ref_vel -= .224;
            // } else if(ego.ref_vel < 49.5){
            //   ego.ref_vel += .224;
            // }

            ego.display("initial ego");
            vehicles.push_back(ego);

            cout << "############### neighbour car position start " << endl;
            for(int i=0; i < sensor_fusion.size(); i++){
              Vehicle other (sensor_fusion[i][0], // car_id
                             sensor_fusion[i][1], // car_x
                             sensor_fusion[i][2], // car_y
                             sensor_fusion[i][3], // vx
                             sensor_fusion[i][4], // vy
                             sensor_fusion[i][5], // car_s
                             sensor_fusion[i][6], // car_d
                             0,                   // acceleration
                             "CS"                 // state
                            );

              if(other.lane != -2) { // cars are not on lane 0,1,2 and not ego car(-1)
                vehicles.push_back(other);
              }

              other.display("orig other car info::");
            }
            ego.display("ego car::");
            cout << "############### car position end display()" << endl;
            map<int ,vector<Vehicle> > predictions;
            for(int i = 0 ; i < vehicles.size(); i++){
              vector<Vehicle> preds = vehicles[i].generate_predictions();
              predictions[vehicles[i].car_id] = preds;
            }

            Vehicle temp_vehicle;
                for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
                    temp_vehicle = it->second[0];

                    temp_vehicle.display("predicted car:: ");
                }


            vector<Vehicle> trajectory = ego.choose_next_state(predictions);
            trajectory[0].display("start projectory");
            trajectory[1].display("end   projector");
            ego.realize_next_state(trajectory);
            ego.display("choosing next state");
            ego.get_next_points(next_x_vals, next_y_vals, trajectory[1].lane);



          	//define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
