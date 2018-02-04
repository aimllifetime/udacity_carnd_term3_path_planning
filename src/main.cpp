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

            for(int i= 0; i<previous_path_x.size();i++)
            {
              prev_path_x.push_back(previous_path_x[i]);
              prev_path_y.push_back(previous_path_y[i]);
            }

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
            vector<Vehicle> trajectory;
            vector<Vehicle> vehicles;

            cout << " ************************ Start here" << endl;
            ego.update_ego(-1, car_x, car_y, car_s, car_d, car_yaw, car_speed,
                           prev_path_x, prev_path_y, end_path_s, end_path_d,
                           map_waypoints_x, map_waypoints_y, map_waypoints_s,
                           map_waypoints_dx, map_waypoints_dy);

            ego.display("after update");
            vehicles.push_back(ego);

            int prev_size = previous_path_x.size();

            cout << "car_x " << car_x << " car_y " << car_y << " car_s " << car_s << " car_d " << car_d << " yaw " << car_yaw << " speed " << car_speed << endl;

            ego.display("initial ego");
            cout << " left size " << prev_size << endl;
            int lane = ego.lane;
            cout << "ego lane " << ego.lane << endl;
            if(prev_size > 0){
              car_s = end_path_s; // previous last point s
            }
            bool too_close = false;
            double front_car_speed = 0.0;
            double ref_vel = 49.5;

            // create ref_v for future use
            for(int i =0; i < sensor_fusion.size(); i++){
              // car in ego car lane
              float d = sensor_fusion[i][6];
              cout << "main::check d " << d << " car_id " << sensor_fusion[i][0] << " ego car_d " << car_d << endl;
              double nei_car_s = sensor_fusion[i][5];
              if( abs(d - car_d) < 3.5 && abs(car_s - nei_car_s) < 1){
                cout << "collision on d " << endl;
                //exit(0);
              }

              if(d < (2 + 4*lane+2) && d > (2+4*lane-2)){
                cout << "found car " << sensor_fusion[i][0] << "on lane "<< lane<< endl;
                double vx = sensor_fusion[i][3]; // x velocity
                double vy = sensor_fusion[i][4]; // y velocity
                double check_speed = sqrt(vx*vx+vy*vy); // magnitude of velocity
                double check_car_s = sensor_fusion[i][5];
                check_car_s += ((double) (prev_size) * .02 * check_speed);
                //front_car_speed = check_speed ;
                cout << "check_car_s " << check_car_s << " ego car s " << car_s << endl;
                if(abs(check_car_s - car_s) < 1){
                  cout << "collision happend" << endl;
                  exit(0);
                }
                if((check_car_s > car_s) && ((check_car_s-car_s) < 30)){
                  // check the car s with car in front us. if the buffer 30 m then slow down or
                  // consider change lane.
                  cout << " car in front of us distance:" << check_car_s-car_s << endl;
                  ref_vel = check_speed; // mph
                  too_close = true;

                  // consider change lane or use JMT to slow down the ego car
                }
              }
            }



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

              other.display("neighbour car info::");
            }
            ego.display("ego car:: <<<<<<<<<<<<<<");
            cout << "############### car position end display()" << endl;

            if(ego.state.compare("LCR") == 0 || ego.state.compare("LCL") == 0) {
              ego.create_lane_change_points(ego.trajectory);
              cout << "ego goal_lane " << ego.goal_lane << " car_d " << car_d << endl;
              if((ego.goal_lane == 0 && abs(car_d - 2.0) < 0.25 ) ||
                 (ego.goal_lane == 1 && abs(car_d - 6.0) < 0.25 ) ||
                 (ego.goal_lane == 2 && abs(car_d - 10.0) < 0.25 ) ){
                ego.state = "KL";
                cout << "lane change completed " << endl;
                // ego.lane_change_end = false;
                // ego.lane_change_start = false;
                // if(ego.state.compare("LCR") == 0) {ego.lane += 1;}
                // else { ego.lane -= 1; }
                ego.goal_lane = ego.lane; // reset the goal_lane

              }


            } else if(ego.state.compare("PLCL") == 0 || ego.state.compare("PLCR") == 0){

              cout << "entering ego state " << ego.state << "create_prep_lane_change_points" << " ego lane "<< ego.lane << endl;

              // map<int ,vector<Vehicle> > predictions;
              // for(int i = 0 ; i < vehicles.size(); i++){
              //   vector<Vehicle> preds = vehicles[i].generate_predictions();
              //   predictions[vehicles[i].car_id] = preds;
              // }
              //
              // Vehicle temp_vehicle;
              // for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
              //   temp_vehicle = it->second[0];
              //   temp_vehicle.display("predicted car:: ");
              // }
              //
              // trajectory = ego.choose_next_state(predictions);
              // trajectory[0].display("start projectory");
              // trajectory[1].display("end   projector");
              // //ego.state = trajectory[1].state;
              // ego.trajectory = trajectory;

              if(ego.state.compare("PLCL") == 0) {
              //if(ego.trajectory[1].lane < ego.lane){
                ego.state = "LCL";
                ego.goal_lane = lane - 1.0;
              } else {
                ego.state = "LCR";
                ego.goal_lane = lane + 1.0;
              }

              ego.create_prep_lane_change_points(ego.trajectory);


              //ego.create_lane_change_points(ego.trajectory);
              // if(trajectory[1].lane < ego.lane){
              //   ego.state = "LCL";
              // } else if(trajectory[1].lane > ego.lane){
              //   ego.state = "LCR";
              // }


            } else if(ego.state.compare("KL") == 0){

              if(too_close){
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

                trajectory = ego.choose_next_state(predictions);
                trajectory[0].display("start projectory");
                trajectory[1].display("end   projector");
                ego.state = trajectory[1].state;
                ego.trajectory = trajectory;
                // if(trajectory[1].lane < ego.lane){
                //   ego.state = "PLCL";
                // }else if (trajectory[1].lane > ego.lane){
                //   ego.state = "PLCR";
                // } else { ego.state = "KL";}

                ego.create_keep_lane_points(ego.lane);
              } else{

                // if(ego.init_speedup) {
                //
                //   ego.create_init_points();
                // }
                // else { ego.create_keep_lane_points(ego.lane);}
                ego.create_keep_lane_points(ego.lane);
              }
            }

            ego.get_next_points(next_x_vals, next_y_vals);


            if(too_close){ // seek to change lane
              double speed_min;
              if(ego.state.compare("KL") == 0 ){
                speed_min = front_car_speed;
              } else {
                speed_min = 35;
              }

              if(ego.ref_vel > ref_vel && ego.ref_vel > speed_min) {
                cout << "*** speed **** ego state " << ego.state << " ego ref_vel " << ego.ref_vel <<
                " ref_vel " << ref_vel << " front_car_speed " << front_car_speed
                <<  " speed_min " << speed_min << endl;

                ego.ref_vel -= .224;
              } else{
                ego.ref_vel += .224;
              }
            } else if (ego.ref_vel < 49.5){
              ego.ref_vel += .224;
            }

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
