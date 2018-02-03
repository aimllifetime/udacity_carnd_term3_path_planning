#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "help.h"
/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int car_id, int lane, float s, float v, float a, string state) {
    this->car_id = car_id;
    this->lane = lane;
    this->car_s = s;
    //
    //this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = -1;

}
Vehicle::Vehicle(int car_id, double car_x, double car_y, double vx, double vy, double car_s, double car_d, double acc,  string state="CS"){



  this->car_id = car_id;
  this->car_x = car_x;
  this->car_y = car_y;
  this->vx = vx;
  this->vy = vy;
  this->car_s = car_s;
  this->car_d = car_d;
  this->acc = acc;
  if(car_d > 0 && car_d <= 4){
    this->lane = 0;
  } else if(car_d > 4 && car_d <= 8){
    this->lane = 1;
  } else if(car_d > 8 && car_d <= 12){
    this->lane = 2;
  } else{
    cout << "found invalid lane " << car_d << endl;
    this->lane = -1; // invalid lane
  }

  double velocity = sqrt(vx*vx + vy*vy); // this is simplified version of v
    this->car_s = car_s;
    this->v = sqrt(vx*vx + vy*vy);
    this->a = acc;
    this->state = state;
    max_acceleration = -1;

}


void Vehicle::update_ego(int car_id, double car_x, double car_y, double car_s, double car_d,
        double yaw, double speed, vector<double> & previous_path_x,
        vector<double> & previous_path_y, double end_path_s, double end_path_d,
        vector<double> & map_waypoints_x,
        vector<double> & map_waypoints_y,
        vector<double> & map_waypoints_s,
        vector<double> & map_waypoints_dx,
        vector<double> & map_waypoints_dy)
{
  cout << "update ego" << endl;
  this->car_id = car_id;
  this->car_x = car_x;
  this->car_y = car_y;
  this->car_s = car_s;
  this->car_d = car_d;
  this->car_yaw = yaw;
  this->speed = speed;
  this->v = speed;
  this->previous_path_x = previous_path_x;
  this->previous_path_y = previous_path_y;
  this->end_path_s = end_path_s;
  this->end_path_d = end_path_d;
  this->map_waypoints_x = map_waypoints_x;
  this->map_waypoints_y = map_waypoints_y;
  this->map_waypoints_s = map_waypoints_s;
  this->map_waypoints_dx = map_waypoints_dx;
  this->map_waypoints_dy = map_waypoints_dy;
  cout << "update_ego:: car_d " << car_d << endl;
  if(car_d > 0.0 && car_d <= 4.0){
    this->lane = 0;
    cout << "update lane to 0" << endl;
  } else if(car_d > 4.0 && car_d <= 8.0){
    this->lane = 1;
    cout << "update lane to 1" << endl;
  } else if(car_d > 8.0 && car_d <= 12.0){
    cout << "update lane to 2" << endl;
    this->lane = 2;
  } else{
    cout << "found invalid lane " << car_d << endl;
    this->lane = -1; // invalid lane
  }

  // debug stop
  //if(sim_step_count ++ > 200){ exit (0); };

  new_path_x.clear();
  new_path_y.clear();
  // update the ego target s to move 50m ahead

  this->goal_s = 5000 ; // car_s + 50;
  //this->s = car_s;
  // if(init_speedup){
  //   this->lane = 1;
  // }
}

void Vehicle::same_lane_speed_control(){
  //
  // cout << "car_speed " << car_speed << endl;
  // cout << "prev_size " << prev_size << endl;
  // bool too_close = false;
  // // create ref_v for future use
  // for(int i =0; i < sensor_fusion.size(); i++){
  //   // car in ego car lane
  //   float d = sensor_fusion[i][6];
  //   cout << "d for car " << d << "car_id" << sensor_fusion[i][0] << endl;
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
  //   ref_vel -= .224;
  // } else if(ref_vel < 49.5){
  //   ref_vel += .224;
  // }

}
void Vehicle:: create_init_points(){
  // JMT to speed up to 50MPH if there is no car in front in 1s

  init_speedup = false;
  cout << "detect the initial on road" << endl;
  cout << "size " << previous_path_x.size() << "map way point size"<< map_waypoints_dy.size() << endl;
  cout << "ego car lane " << lane << " car_s " << car_s << " car_d " << car_d << endl;
  JMT_CFG init_speed_up;
  init_speed_up.start = {car_s, 0, 10};
  init_speed_up.end   = {car_s + 122.5125, 49.5, 10};
  lane = 1.0;
  vector< double > jmt = JMT(init_speed_up.start, init_speed_up.end, 4.95);
  double t_delta = 0.02;
  vector<double> s_init;
  for(int i = 0; i < 247; i++){ // calculate the first 50 points to get speed up to 50MPH.
    double t = ((double)(i)) * t_delta;
    double s = (jmt[0] + jmt[1]*t + jmt[2]*t*t + jmt[3]*t*t*t + jmt[4] *t*t*t*t + jmt[5]*t*t*t*t*t);
    cout << " New s " << s << " input s " << car_s;
    vector<double> wpt = getXY(s, car_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    cout << " point x: " << wpt[0] << " y: " << wpt[1] << endl;
    new_path_x.push_back(wpt[0]);
    new_path_y.push_back(wpt[1]);
  }
  cout << "exit of create_init_points" << endl;
}



void::Vehicle::create_keep_lane_points(double lane){

  //same_lane_speed_control();

  cout << "not in initial block" << endl;
  int prev_size = previous_path_x.size();
  cout << "previous_path_x is " << previous_path_x.size() << endl;
  if(prev_size > 0){
    car_s = end_path_s; // previous last point s
  }
  // create a list of widely spaced(x, y) waypoints, evenly spaced at 30m
  // later will interoplate these waypoints with a spline and fill it in with more points that control speed
  vector<double> ptsx;
  vector<double> ptsy;

  // reference x, y, yaw states// either will reference the starting points as where the car is or at previous end states
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  cout << "car_s " << car_s << " car_x " << car_x << " car_y " << car_y << " yaw " << car_yaw << endl;
  // if previoius size is almost empty, use the car as starting reference
  if(prev_size < 2){
    // use two points that make the path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);

  } else {
    // redefine reference state as previous path end points
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];

    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use two points that make the path tangent to the previous path's end point

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  cout << "spline lane " << lane << endl;
  // in Frenet add evenly 30m spaced points ahead of the startig reference
  vector<double> next_wp0 = getXY(car_s+30,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(car_s+60,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(car_s+90,(2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for(int i = 0; i < ptsx.size(); i++){
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i]-ref_x;
    double shift_y = ptsy[i]-ref_y;
    ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
    ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));

    cout << "debug spline" << ptsx[i] << " " << ptsy[i] << endl;
  }

  // create a spline
  tk::spline s;

  // set(x, y) points to the spline
  s.set_points(ptsx, ptsy);
  cout << "after spline initialiation" << endl;
  //int next_wp = NextWaypoint(car_x,car_y, car_yaw, map_waypoints_x, map_waypoints_y);

  // std::cout << "next_wp " << 0 << std::endl;
  // std::cout << "previous_path_x size = " << previous_path_x.size() << std::endl;

  for(int i = 0; i < previous_path_x.size(); i++){
    new_path_x.push_back(previous_path_x[i]);
    new_path_y.push_back(previous_path_y[i]);
  }

  // calculate how to break up spline points so that we can travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x)*target_x+ target_y*target_y);

  double x_add_on = 0.0;

  // fill up the rest of our path planner after filling it with previous points. here we always output 50 points
  for(int i = 1; i <= 50-previous_path_x.size(); i++){
    double N;

    N = (target_dist/(.02*ref_vel/2.24));  // mph to meter/h for 2.24

    double x_point = x_add_on + target_x/N;
    double y_point = s(x_point);

    x_add_on = x_point;
    double x_ref = x_point;
    double y_ref = y_point;

    // rotate back to normal after rotatig it early
    x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
    y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    new_path_x.push_back(x_point);
    new_path_y.push_back(y_point);


  }

}

void Vehicle::create_prep_lane_change_points(vector<Vehicle> trajectory){

  create_keep_lane_points(goal_lane);
  return;
  // // calcuate the points for 2 seconds. and append them in the LCR/LCL state. so this only calculate once during lane change
  // int size = previous_path_x.size();
  // cout << "trajectory size " << trajectory.size() << endl;
  // if(!lane_change_start) {
  //   if(size > 0) { car_s = end_path_s;}
  //   lane_change_goal = car_s + 50;
  //   lane_change_start = true;
  //   for(int i =0; i < size; i++){
  //     cout << "simulator point remains:: size " << i << " car_d " << car_d << " debug x " << previous_path_x[i] << " y " << previous_path_y[i] << endl;
  //   }
  //   //vector<double> starting_s_d = getFrenet(previous_path_x[size-1], previous_path_y[size-1], map_waypoints_x, map_way_points_y)
  //   cout << "here 1" << endl;
  //   JMT_CFG change_lane_d;
  //   change_lane_d.start = {car_d, 0, 0};  // d, d dot, d dotdot
  //   cout << "here 2" << "car_d " << car_d << " target lane " << trajectory[1].lane << endl;
  //
  //   //double lane = trajectory[1].lane;
  //   //change_lane_d.end = {2.0*(double)trajectory[1].lane+2.0,  1.0, 0};
  //   double change_lane_to;
  //   if(state.compare("PLCL") == 0){ change_lane_to = lane - 1.0; } else{change_lane_to += 1.0;}
  //   change_lane_d.end = {2.0 * change_lane_to + 2.0 ,  0.001, 0};
  //   cout << "end lane " << change_lane_d.end[0];
  //   cout << " before JMT called" << endl;
  //   vector< double > jmt = JMT(change_lane_d.start, change_lane_d.end, 2);
  //   cout << "after JMT called" << endl;
  //   double t_delta = 0.02;
  //
  //   vector<double> d;
  //   for(int i = 0; i < 100; i++){ // calculate rest of points to change lane
  //     double t = ((double)(i)) * t_delta;
  //     double dt = (jmt[0] + jmt[1]*t + jmt[2]*t*t + jmt[3]*t*t*t + jmt[4] *t*t*t*t + jmt[5]*t*t*t*t*t);
  //     cout << "JMT d offset " << " i = " << i << " dt "<< dt << endl;
  //     d.push_back(dt);
  //   }
  //
  //   cout << " d delta JMT is done " << " car_s " << car_s << " goal_s " << goal_s << " ref_vel " << ref_vel << endl;
  //   JMT_CFG change_lane_s;
  //   change_lane_s.start = {car_s, ref_vel, 0};
  //   change_lane_s.end = {(double)(car_s + 50.0), ref_vel, 0};
  //   jmt = JMT(change_lane_s.start, change_lane_s.end, 2);
  //
  //
  //   for(int i = 0; i < 100; i++){ // calculate rest of points to change lane
  //     double t = ((double)(i)) * t_delta;
  //     double s = (jmt[0] + jmt[1]*t + jmt[2]*t*t + jmt[3]*t*t*t + jmt[4] *t*t*t*t + jmt[5]*t*t*t*t*t);
  //     vector<double> wpt = getXY(s,(d[i]), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  //     cout << "calculating the chane lane point " << wpt[0] << "  " << wpt[1] << endl;
  //     lane_change_ptx.push_back(wpt[0]);
  //     lane_change_pty.push_back(wpt[1]);
  //   }
  // }
  // lane_change_offset_idx = 0;
  cout << "create_prep_lane_change_points is done" << "points created" << lane_change_pty.size()<< endl;
}

void Vehicle::create_lane_change_points(vector<Vehicle> trajectory){

  create_keep_lane_points(goal_lane);
  // int prev_size = previous_path_x.size();
  //
  // cout << "create_lane_change_points prev_size " << prev_size << "lane_change_offset_idx" << lane_change_offset_idx << endl;
  //
  // for(int i = 0; i < 50 - prev_size; i++ ){
  //   if(lane_change_offset_idx < 100) {
  //     new_path_x.push_back(lane_change_ptx[lane_change_offset_idx]);
  //     new_path_y.push_back(lane_change_pty[lane_change_offset_idx]);
  //     lane_change_offset_idx ++;
  //   }
  // }
  //
  //
  // if(lane_change_offset_idx == 100){
  //   cout << "all points for lane change are used up" << endl;
  //   lane_change_end = true;
  //   //if(new_path_x.size() )
  // }

}

void Vehicle::get_next_points(vector<double> &next_x_vals, vector<double> & next_y_vals){
  //cout << "new path_x size " << new_path_x.size() << endl;
  for(int i = 0; i < new_path_x.size(); i++){
    next_x_vals.push_back(new_path_x[i]);
    next_y_vals.push_back(new_path_y[i]);
    cout << "get_next_points: " << new_path_x[i] << "   : " << new_path_y[i] << endl;
  }
}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    // this only apply to the ergo vehicle. other vehicle skips
    */
    vector<vector<Vehicle>> final_trajectories;
    vector<Vehicle> non_ego;
    if(this->car_id == this->ego){
      vector<string> states = successor_states();

      float cost;
      vector<float> costs;
      vector<string> final_states;
      cout << "choose_next_state called with current state" << state << endl;
      for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        cout << " successor_states " << *it << endl;
          vector<Vehicle> trajectory = generate_trajectory(*it, predictions);

          if (trajectory.size() != 0) {

              cost = calculate_cost(*this, predictions, trajectory);
              costs.push_back(cost);
              cout << "trajectory size " << trajectory.size() << endl;
              cout << "next state " << *it << endl;
              cout << "cost "  << cost << "new lane "<< trajectory[1].lane << endl;
              trajectory[0].display("trajectory 0: ");
              trajectory[1].display("trajectory 1: ");
              final_trajectories.push_back(trajectory);
          }
      }

      vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
      int best_idx = distance(begin(costs), best_cost);
      cout << "best cost " << *best_cost << "lane in trajectory" << final_trajectories[best_idx][1].lane << "state " << final_trajectories[best_idx][1].state << endl;
      return final_trajectories[best_idx];
    } else {
      return non_ego;
    }
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    cout << "current state " << state << endl;
    if(state.compare("KL") == 0) {
        if(lane == 1 || lane == 2) { states.push_back("PLCL");}
        if(lane == 0 || lane == 1) { states.push_back("PLCR"); }
        cout << "pushed other two PLCL and PLCR" << endl;
    } else if (state.compare("PLCL") == 0) {
        //if (lane != lanes_available - 1) { // change to look forward not like one in class
        if (lane > 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (state.compare("PLCR") == 0) {
        if (lane < 2) { // we total have 3 lane, 0, 1, 2. other two 0, 1 can change lane to right.
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    //If state is "LCL" or "LCR", then just return "KL"
    // debug:
    cout << "vehicle car_id " << car_id << ": successor_states=  "<<  state << endl;
    for (int i = 0; i < states.size(); i++){
      cout << states[i] << endl;
    }
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    float max_velocity_accel_limit = this->max_acceleration + this->v;
    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
      vehicle_ahead.display("Debug vehicle ahead found::");
      cout << "distance "<< vehicle_ahead.car_s - car_s << endl;
        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            // todo: check if my speed is fast or slow than others. adjust the speed smoothly.
            new_velocity = vehicle_ahead.v; //must travel at the speed of traffic, regardless of preferred buffer

        } else {
            float max_velocity_in_front = (vehicle_ahead.car_s - this->car_s - this->preferred_buffer) + vehicle_ahead.v - 0.5 * (this->a);
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
    }

    new_accel = new_velocity - this->v; //Equation: (v_1 - v_0)/t = acceleration
    new_position = this->car_s + new_velocity + new_accel / 2.0;
    return{new_position, new_velocity, new_accel};

}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    float next_pos = position_at(1);
    vector<Vehicle> trajectory = {Vehicle(this->car_id, this->lane, this->car_s, this->v, this->a, this->state),
                                  Vehicle(this->car_id, this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    vector<Vehicle> trajectory = {Vehicle(this->car_id, lane, this->car_s, this->v, this->a, state)};
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->car_id, this->lane, new_s, new_v, new_a, "KL"));
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    cout << "prep_lane_change_trajectory is called with state: " << state << endl;
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->car_id, this->lane, this->car_s, this->v, this->a, this->state)};
    vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];

    } else {
        vector<float> best_kinematics;
        vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    //trajectory.push_back(Vehicle(this->car_id, this->lane, new_s, new_v, new_a, state));
    trajectory.push_back(Vehicle(this->car_id, new_lane, new_s, new_v, new_a, state));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    cout << "lane_change_trajectory called " << "this lane " << this->lane << "new_lane " << new_lane << endl;
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.car_s == this->car_s && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->car_id, this->lane, this->car_s, this->v, this->a, this->state));
    vector<float> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(this->car_id, new_lane, kinematics[0], kinematics[1], kinematics[2], state));
    return trajectory;
}

void Vehicle::increment(int dt = 1) {
	this->car_s = position_at(dt);
}

float Vehicle::position_at(int t) {
    return this->car_s + this->v*t + this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.car_s < this->car_s && temp_vehicle.car_s > max_s) {
            max_s = temp_vehicle.car_s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int min_s = car_s + 50; //this->goal_s;
    bool found_vehicle = false;
    //int cloest_s = 9999;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        // cout << "prediction car_id " << it->first << " s " << temp_vehicle.s << " lane " << temp_vehicle.lane << endl;
        temp_vehicle.display("prediction vehicle head:: ");
        if (temp_vehicle.lane == this->lane && temp_vehicle.car_s > this->car_s && temp_vehicle.car_s < min_s && temp_vehicle.car_id != this->car_id) {
            min_s = temp_vehicle.car_s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
    cout << "generate prediction for car" << "horizon: " << horizon << endl;
    this->display("generate prediction before :: ");
	  vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
      float next_s = position_at(i);
      float next_v = v;  //todo check if the v is constant for ego. can not tell for other neighbour cars
      if (i < horizon-1) {
        next_v = position_at(i+1) - car_s;
      }
      Vehicle pred;
      //predictions.push_back(Vehicle(this->car_id, this->car_x, this->car_y, this->vx, this->vy, this->car_s, this->car_d, 0, "CS" );
      // this->lane, next_s, next_v, 0));


      pred.lane = this->lane;
      pred.car_s = next_s;
      pred.v = next_v;
      pred.car_d = this->car_d;
      pred.car_id = this->car_id;
      pred.speed = this->speed;
      pred.display("generate predictions new :: " );
      predictions.push_back(pred);
  	}

    return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    if(this->state.compare("LCR") == 0 || state.compare("LCL") == 0){

    }
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;

    cout << " ***************** lane selection for next state" << endl;
    cout << " current state: " << state << "; new state:" << next_state.state << endl;


    if(state.compare("PLCL") == 0 || state.compare("PLCR") == 0 ){
      //create_prep_lane_change_points(trajectory);
      create_keep_lane_points(lane);
    }
    else if(state.compare("KL") == 0 ) { // use the prejectory
      create_keep_lane_points(lane);// need to adjust the speed based on jmt
    } else if( state.compare("LCR") == 0 || state.compare("LCL") == 0){
      create_prep_lane_change_points(trajectory);
      create_lane_change_points(trajectory);
    }

    // this->lane = next_state.lane;
    //
    // this->car_s = next_state.car_s;
    // this->v = next_state.v;
    // this->a = next_state.a;
    // this->lane = next_state.lane;
    // this->car_d = next_state.car_d;
}

void Vehicle::configure(vector<int> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}

void Vehicle::display(string text){
  cout << text << " state: " << state << " car_id " << car_id
       << " car_d " << car_d
       << " s= " << car_s << " lane = " << lane << " v " << v
          << " vx " << vx << " vy " << vy << endl;
  // cout << "car_id " << car_id << endl;
  // cout << "car_x "  << car_x << " car_y " << car_y << endl;
  // cout << "vx "  << vx << " vy " << vy << endl;
  // cout << "car_s "  << car_s << " car_d " << car_d << endl;
}
