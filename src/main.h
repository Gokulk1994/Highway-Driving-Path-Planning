#ifndef MAIN_H
#define MAIN_H

#include <string>
#include <vector>


#define MAX_COST (double) 999.0         // default cost value set to maximum
#define MOVE_LEFT (string) "MoveLeft"   // Move left state variable
#define MOVE_RIGHT (string) "MoveRight" // move right state variable

/*
All information related to the neighbouring vehicles obtained from the map and sensor fusion
*/

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

/*
Placeholder to hold all necessary vehicle data
*/
struct Vehicle{
  double s 		= 0.0;
  double d 	    = 0.0;
  double vel 	= 0.0;
  int lane	    = 0;
};

/*
Sigmoid function, converts range between 0 to 1
*/
double sigmoid(double x)
{
  return (1.0 / (1 + exp(-x)));
}
#endif