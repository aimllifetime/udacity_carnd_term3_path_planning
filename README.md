# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

Sample video is here: [report_video](./report_good.mov)

Files are include in the submission:
  src/main.cpp
  src/cost.cpp
  src/cost.h
  src/vehcicle.cpp
  src/vehicle.h
  src/help


Implementation details:

Path planning:

Use the state machine to track states between "KL", "PLCL", "PLCR", "LCR", "LCL"

Psudo code is as follows: (this is in main.cpp)

create list of vehicle on lanes of same side of road
check to see if there is any car in front and closer within 25 miles for 1 second time
if yes, generate projectory
for each projector, calculate the cost: inefficiency, collision, goal to distance
choose lowest trajectory. prepare change lane.
state change to "PLCL", or "PLCR" dependes on the trajectory.
in Prepare change lane state, update the "goal lane" and then change state to "LCL" or "LCR"
In "LCL" or "LCR", calculate the new path and determine if lane change to target lane.
else
Keep currentlane
 
