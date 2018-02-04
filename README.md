# udacity_carnd_term3_path_planning

# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

Sample video is here: [report_video](./report_good.mov)

Files are include in the submission:<br/>
&nbsp; &nbsp; src/main.cpp <br/>
&nbsp; &nbsp; src/cost.cpp <br/>
&nbsp; &nbsp; src/cost.h <br/>
&nbsp; &nbsp; src/vehcicle.cpp <br/>
&nbsp; &nbsp; src/vehicle.h <br/>
&nbsp; &nbsp; src/help.h<br/>


# Implementation details:<br/>
Path planning:<br/>
&nbsp; &nbsp;     Use the state machine to track states between "KL", "PLCL", "PLCR", "LCR", "LCL"<br/>
&nbsp; &nbsp;     Psudo code is as follows: (this is in main.cpp)<br/>
&nbsp; &nbsp;       create list of vehicle on lanes of same side of road<br/>
&nbsp; &nbsp;       check to see if there is any car in front and closer within 25 miles for 1 second time <br/>
&nbsp; &nbsp;       if yes, generate projectory.<br/>
&nbsp; &nbsp;       for each projector, calculate the cost: inefficiency, collision, goal to distance<br/>
&nbsp; &nbsp;       choose lowest trajectory. prepare change lane.<br/>
&nbsp; &nbsp;       state change to "PLCL", or "PLCR" dependes on the trajectory.<br/>
&nbsp; &nbsp;       in Prepare change lane state, update the "goal lane" and then change state to "LCL" or "LCR"<br/>
&nbsp; &nbsp;       In "LCL" or "LCR", calculate the new path and determine if lane change to target lane.<br/>
<br/>
# Prediction of vehicles:<br/>
&nbsp; &nbsp;     for every vehicle on the same side of road, calculate the next positions apart every 0.02 second.<br/>
&nbsp; &nbsp;     that says, there are 50 vehicles for one car predicted postistion. these vehicles are use to calculate if there is collison when possible lane change. <br/>
<br/>
# Trajectory Algorithm:<br/>
&nbsp; &nbsp; The alrogithm to calculate the next 50 points is using the spline alrogithm, the ego vehicle state determine to use the current lane or target lane to change.<br/>
<br/>
# Cost function:(in cost.cpp)<br/>
&nbsp; &nbsp;     * inefficiency cost * : calculate the lane change efficiency as follows:<br/>
&nbsp; &nbsp;         cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;<br/>
&nbsp; &nbsp;      *  goal_distance_cost * : calculate the cost to goal distance as follows:<br/>
&nbsp; &nbsp;          1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"] - data["final_lane"]) / distance));<br/>
&nbsp; &nbsp;      *  collision_cost *: calcualte if any collision happens on current lane or predicted lane.<br/>
&nbsp; &nbsp;          for collision on current lane, add cost "COLLISION"<br/>
&nbsp; &nbsp;          for collison on predicted target lane, add 2*"COLLISION"<br/>
&nbsp; &nbsp;          in this case, if change lane cause collison, vehicle will keep on current lane.<br/>
&nbsp; &nbsp;          collision is measured if changed lane for ego car, the distance between ego and other car is less than 30m. this 30m distance guardband is also experimental number and can be better determined by current speed and other car speed as well.<br/>

<br/>
# Speed adjustment:<br/>
 &nbsp; &nbsp;    if car is in "KL" and no way to change lane such as the cost of stay in current lane is minimum. then slow down the ego speed to front car speed plus 14m/s. this additional 14m/s is to avoid car to slow down too much and miss the chance to do lane change. of course, this number is not perfect, will need better methmatical calculation between current car speed and front car speed. this is left to future.if car in "KL" and speed is lower than 49.5m/s then add 0.224 every sim tick.
<br/>

Note that Vehicle.cpp from class quiz is not naturally fit into this project. in the quiz, the car change lane directly and predicted position in unit of second. it needs a lots of additional change to fit into this project. such as, lane determination, lane numbering from 0, 1, 2, prepare the lane change vehicles, next state logic change.<br/>
<br/>
Notice some case might have collision. the best parameter to adjust the two collision distance guardband, 25m used in main.cpp and 30m used in collision_cost.(future work to have better calculation).<br/>
