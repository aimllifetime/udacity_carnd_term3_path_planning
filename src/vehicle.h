#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", +1}, {"PLCR", +1}};

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int car_id;
  double car_x;
  double car_y;
  double vx;
  double vy;
  double car_s;
  double car_d;
  double acc; // acceleration.

  int ego = -1;

  vector<double> previous_path_x;
  vector<double> previous_path_y;
  vector<double> new_path_x;
  vector<double> new_path_y;
  double car_yaw;
  double speed; // this is for ego car from sensor
  double end_path_s;
  double end_path_d;

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  bool init_speedup = true;

  int L = 1;

  int preferred_buffer = 30; // impacts "keep lane" behavior.

  double lane;

  //int s;

  float v;

  float a;

  float target_speed = 49.5;

  int lanes_available;

  float max_acceleration = 5;

  int goal_lane;

  int goal_s = 50; // from current position s to future s = 50 to see if there is any car in front us

  double lane_change_goal = -1;
  bool lane_change_start = false;
  bool lane_change_end = false;
  vector<double> lane_change_ptx;
  vector<double> lane_change_pty;
  int lane_change_offset_idx = 0;
  double ref_vel = 49.5; // MPH
  string state;

  int sim_step_count = 0;
  vector<Vehicle> trajectory;

  /**
  * Constructor
  */
  Vehicle();

  Vehicle(int car_id, int lane, float s, float v, float a, string state="CS");
  Vehicle(int car_id, double car_x, double car_y, double vx, double vy,
          double car_s, double car_d, double acc,  string state);
  void update_ego(int car_id, double car_x, double car_y, double car_s, double car_d,
          double yaw, double speed, vector<double> &previous_path_x,
          vector<double> &previous_path_y, double end_path_s, double end_path_d,
          vector<double> & map_waypoints_x,
          vector<double> & map_waypoints_y,
          vector<double> & map_waypoints_s,
          vector<double> & map_waypoints_dx,
          vector<double> & map_waypoints_dy);


  void create_keep_lane_points(double lane);
  void create_prep_lane_change_points(vector<Vehicle> trajectory);
  void create_lane_change_points(vector<Vehicle> trajectory);
  void create_init_points();
  /**
  * Destructor
  */
  virtual ~Vehicle();
  void display(string text);
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  void increment(int dt);

  float position_at(int t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  vector<Vehicle> generate_predictions(int horizon=2);

  void realize_next_state(vector<Vehicle> trajectory);

  void get_next_points(vector<double> &next_x_vals, vector<double> & next_y_vals);

  void configure(vector<int> road_data);

};

#endif
