#ifndef MAIN_H
#define MAIN_H

#include <string>
#include <vector>


struct AllRoadVehicleDetails{
  bool Vehicle_Ahead		= false;
  bool Vehicle_Behind		= false;
  bool Vehicle_Left		    = false;
  bool Vehicle_Left_Behind	= false;
  bool Vehicle_Right		= false;
  bool Vehicle_Right_Behind = false;
  double Near_Veh_Left 		= 0.0;
  double Near_Veh_Right     = 0.0;
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