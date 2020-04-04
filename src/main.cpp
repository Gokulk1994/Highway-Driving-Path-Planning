#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <cmath>
#include "main.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::map;


json GetAllWayPointsMsg(vector<double> ptsx, vector<double> ptsy,vector<double> previous_path_x,vector<double> previous_path_y, double refVelocity ,
                       double ref_x, double ref_y, double ref_yaw)
{
  tk::spline s;          
  s.set_points(ptsx,ptsy);         
  vector<double> next_x_vals;
  vector<double> next_y_vals;  
  json msgJson;
  

  for(int i=0; i<previous_path_x.size(); i++)
  {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
    

  
  double target_x = 30.0;
  double target_y = s(target_x);
  double dist     = sqrt((target_x*target_x) + (target_y*target_y));

  double x_add_on = 0;

  for(int i = 1; i < 50 - previous_path_x.size() ; i++)
  {

    double N = dist / ((0.02 * refVelocity) / 2.24); 
    double x_point = x_add_on + (target_x / N);
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;     

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point); 
  }
  
  msgJson["next_x"] = next_x_vals;
  msgJson["next_y"] = next_y_vals;
  
  return msgJson;
}


// Get all possible lane transitions from current state.
vector<string> GetAllPossibleStates(int lane)
{
  vector<string> states;
  
  if(lane == 0)
  {
    states.push_back("MoveRight");
  }
  else if(lane == 1)
  {
   	states.push_back("MoveRight");
    states.push_back("MoveLeft");
  }
  else if(lane == 2)
    states.push_back("MoveLeft");
  else
    states.push_back("InvalidLane");
  
  return states;
}


AllRoadVehicleDetails Check_Neigbour_VehicleDetails(double car_s,int lane,vector<Vehicle> &VehicleData)
{
  AllRoadVehicleDetails VehicleDetails;

  for(int i =0; i < VehicleData.size(); i++)
  {

    double check_car_s    = VehicleData[i].s;
    double check_car_lane = VehicleData[i].lane;
    
    if(check_car_lane == lane)
    {
      if((check_car_s > car_s) && ((check_car_s - car_s) < 30))
      {
        VehicleDetails.Vehicle_Ahead = true;     
      }
    }
    else if((check_car_lane == lane + 1) && (lane + 1 < 3))
    {
      if(abs(check_car_s - car_s) < 40)
      {
        VehicleDetails.Vehicle_Right = true;
      }
      if(VehicleDetails.Near_Veh_Right > check_car_s)
      {
        VehicleDetails.Near_Veh_Right = check_car_s;
      }
    }
    else if ((check_car_lane == lane - 1) && (lane - 1 > -1))
    {
      if(abs(check_car_s - car_s) < 40)
      {
        VehicleDetails.Vehicle_Left = true;
      }
      if(VehicleDetails.Near_Veh_Left > check_car_s)
      {
        VehicleDetails.Near_Veh_Left = check_car_s;
      }
    }
  }

  return VehicleDetails;
}

Vehicle CheckTransition(double car_s,int lane,double refVelocity,vector<Vehicle> &VehicleData)
{
  
  AllRoadVehicleDetails VehicleDetails = Check_Neigbour_VehicleDetails(car_s,lane,VehicleData);
  vector<string> states =  GetAllPossibleStates(lane);
  
  if(VehicleDetails.Vehicle_Ahead){            

    if(!VehicleDetails.Vehicle_Left && lane != 0)
      lane = lane-1;
    else if (!VehicleDetails.Vehicle_Right && lane != 2)
      lane = lane+1;
    else
      refVelocity -= 0.224;                         
  }
  else if(refVelocity < 49.5){
    refVelocity += 0.224;           
  } 
  
  Vehicle VehNewUpdate;
  
  VehNewUpdate.lane = lane;
  VehNewUpdate.vel  = refVelocity;
  
  return VehNewUpdate;
    
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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  int lane = 1;
  double refVelocity = 0.0;
  
  h.onMessage([&lane,&refVelocity,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
		  
          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
		  vector<Vehicle> VehicleData;
          json msgJson;          

          int prev_path_Size = previous_path_x.size();
		            
          for(int i =0; i< sensor_fusion.size(); i++)
          {
    		double vx  = sensor_fusion[i][3];
    		double vy  = sensor_fusion[i][4];
            
            Vehicle Veh;
                      
            Veh.vel    = sqrt(vx*vx + vy*vy);
            Veh.s      = sensor_fusion[i][5];
            Veh.s 	  += ((double)prev_path_Size*0.02*Veh.vel);
            Veh.d      = sensor_fusion[i][6];
            Veh.lane   = floor(Veh.d / 4);
            
            VehicleData.push_back(Veh);
          }
                                
          if (prev_path_Size > 0) {
            car_s = end_path_s;
          }
          
          Vehicle newParam = CheckTransition(car_s,lane,refVelocity,VehicleData);          
          lane = newParam.lane;
          refVelocity = newParam.vel;
           
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          if(prev_path_Size < 2){
            
            double preve_car_x  = car_x - cos(car_yaw);
            double preve_car_y  = car_y - sin(car_yaw);
            
            ptsx.push_back(preve_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(preve_car_y);
            ptsy.push_back(car_y);
            
          }
          
          else
          {           
            ref_x = previous_path_x[prev_path_Size - 1];
            ref_y = previous_path_y[prev_path_Size - 1];
            
            double ref_prev_x = previous_path_x[prev_path_Size - 2];
            double ref_prev_y = previous_path_y[prev_path_Size - 2];
            
            ref_yaw = atan2( ref_y - ref_prev_y,ref_x - ref_prev_x);
            
            ptsx.push_back(ref_prev_x);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_prev_y);
            ptsy.push_back(ref_y);
            
          }
          
          
          vector<double> next_wp0 = getXY(car_s + 30 ,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y); 
          vector<double> next_wp1 = getXY(car_s + 60 ,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y); 
          vector<double> next_wp2 = getXY(car_s + 90 ,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y); 
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
            
		
          for(int i = 0; i < ptsx.size(); i++){
            
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            
            ptsx[i] = (shift_x * cos(0-ref_yaw)) - (shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw)) + (shift_y * cos(0-ref_yaw));
            
          }
          
		  msgJson = GetAllWayPointsMsg(ptsx, ptsy, previous_path_x, previous_path_y, refVelocity, ref_x, ref_y, ref_yaw);


          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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