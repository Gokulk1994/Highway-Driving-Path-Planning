#ifndef MAIN_H
#define MAIN_H

#include <string>
#include <vector>


#define MAX_COST (double) 999.0
#define MOVE_LEFT (string) "MoveLeft"
#define MOVE_RIGHT (string) "MoveRight"

struct AllRoadVehicleDetails{
  bool Vehicle_Ahead		= false;
  bool Vehicle_Behind		= false;
  bool Vehicle_Left		    = false;
  bool Vehicle_Left_Behind	= false;
  bool Vehicle_Right		= false;
  bool Vehicle_Right_Behind = false;
  
  double Near_Veh_Left 		= 6945.554;
  double Near_Veh_Right     = 6945.554;
  
  double VehAhead_s_diff    = 6945.554;
  double VehBehind_s_diff   = 6945.554;
  
  double VehAhead_Vel       = 0.0;
  
};

struct Vehicle{
  double s 		= 0.0;
  double d 	    = 0.0;
  double vel 	= 0.0;
  int lane	    = 0;
};

struct SDVehicle{
  double s 		= 0.0;
  double d 	    = 0.0;
  double vel 	= 0.0;
  double ref_x  = 0.0;
  double ref_y  = 0.0;
  double yaw    = 0.0;
  int lane	    = 0;
  
  vector<double> previous_path_x;
  vector<double> previous_path_y;
  
};

#endif