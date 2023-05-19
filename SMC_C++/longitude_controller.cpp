/*******************************************************************/
/*                      Author: xubiao                             */
/*                     Contact: xubiao@hnu.edu.cn                  */
/*                 Last update: 2019-08-16                         */
/*******************************************************************/


#include "longitude_controller.h"
#include "float.h"
#include "math.h"
#include "interp.h"
#include "kalman.h"
#include "fuzzy_ctr.h"
#include "config_io.h"

using hivelab_config_io::Config;

// func:  Constructor Initialize variables.
longitude_control::LongitudeController::LongitudeController():kBrakeEmergency(Config::Get<float64>("LON_CONTROLLER","brake_emergency")),
                                           kBrakeShiftGear(Config::Get<float64>("LON_CONTROLLER","brake_shift_gear")),
                                           kDistanceDestination(Config::Get<float64>("LON_CONTROLLER","distance_destination")),
                                           kForwardPreviewNum(Config::Get<int32>("LON_CONTROLLER","forward_preview_num")),
                                           kBackwardPreviewNum(Config::Get<int32>("LON_CONTROLLER","backward_preview_num")),
                                           kGainAcce(Config::Get<float64>("LON_CONTROLLER","gain_acce")),
                                           kAcceMax(Config::Get<float64>("LON_CONTROLLER","acce_max")),
                                           kAcceMin(Config::Get<float64>("LON_CONTROLLER","acce_min")),
                                           kGainTe(Config::Get<float64>("LON_CONTROLLER","gain_thr")),
                                           kGainTb(Config::Get<float64>("LON_CONTROLLER","gain_brk")),
                                           kControlPeriod(Config::Get<float64>("LON_CONTROLLER","control_period")),
                                           kCriticalSpeed(Config::Get<float64>("LON_CONTROLLER","critical_speed")),
                                           kAcceTolerant(Config::Get<float64>("LON_CONTROLLER","acce_tolerant")),
                                           kDeceTolerant(Config::Get<float64>("LON_CONTROLLER","dece_tolerant")),
                                           kThottleMaxChange(Config::Get<float64>("LON_CONTROLLER","thottle_max_change")),
                                           kBrakeMaxChange(Config::Get<float64>("LON_CONTROLLER","brake_max_change")),
                                           kBrakeMax(Config::Get<float64>("LON_CONTROLLER","brake_max")),
                                           kDeltaBrake(Config::Get<float64>("LON_CONTROLLER","low_speed_delta_brake")),
                                           kStopDeltaBrake(Config::Get<float64>("LON_CONTROLLER","stop_speed_delta_brake")),
                                           kBackwardSpeed(Config::Get<float64>("LON_CONTROLLER","backward_speed"))
                                           
{
  integral_speed_error = 0.0;
  speed_desire = 0.0;
  acc_desire = 0.0;
  brake_desire = 0.0; 
  throttle_desire = 0.0;
  preview_x_rear = 0.0 ;
  preview_x_rear = 0.0 ;
  preview_speed = 0.0 ;
  drive_flag_desire = longitude_control::BRAKE;
  high_low_control_flag = 0;
  integral_temp_spd =0;
}

longitude_control::LongitudeController::~LongitudeController()
{
}

// func:   Calculation control command.
// input:  control parameters recieve from decision_path.
// output: calculate desire throttle and acceleration drive_flag(drive mode).
int32 longitude_control::LongitudeController::CalLongitueCommand(const std::vector<WayPointStruct> way_point_,  // path.
                                              const float64 vehicle_x0,                         // vehicle rear axle position x.
                                              const float64 vehicle_y0,                         // vehicle rear axle position y.
                                              const float64 vehicle_z0,
                                              const float64 vehicle_yaw_angle,                  // vehicle position yaw angle, rad.
                                              const float64 vehicle_pitch_angle,                // the road slope from gnss, rad.
                                              const float64 vehicle_speed_value,                // vehicle speed, m/s, forward is +, back is -.
                                              const float64 vehicle_acceleration,               // vehicle acceleration.
                                              const float64 vehicle_throttle,                   // throttle angle, deg.
                                              const float64 vehicle_brake,                      // brake angle/pressure, deg/MPa.
                                              const bool vehicle_gear_flag0,                    // gear = D , flag=1.
                                              const bool gnss_updated_,
                                              const bool station_stop_,                                      
                                              float64 &throttle_value,                          // throttle angle, deg.
                                              float64 &brake_value,                             // brake pressure/angle, MPa/deg.
                                              DriveFlag &drive_flag,                            // control mode . 0 is brake control,1 is accelerator control.
                                              int32 &near_point_id                              // near point id.                  
                                              )                             
{ 
  way_point.clear();
  way_point.assign(way_point_.begin(),way_point_.end());
  yaw_angle = vehicle_yaw_angle;
  x_rear = vehicle_x0;               // rear axle
  y_rear = vehicle_y0;               // rear axle 
  z_rear = vehicle_z0;
  static float64 last_x_rear = vehicle_x0;               // rear axle
  static float64 last_y_rear = vehicle_y0;               // rear axle 
  static float64 last_z_rear = vehicle_z0;

  pitch_angle = vehicle_pitch_angle;   // the road slope from gnss, rad.         
  speed = vehicle_speed_value;
  acceleration = vehicle_acceleration;
  throttle = vehicle_throttle;
  brake = vehicle_brake;
  vehicle_gear_flag = vehicle_gear_flag0;
  std::cout << "\033[32m[brake] : \033[0m"<< brake << std::endl;

  int32 id_nearest_point = FindNearestPoint();  // Returns the ID of the nearest point.
  speed_error = way_point[id_nearest_point].velocity - speed;
  road_slope = CalCulateSlope(); 
  static float64 lastpoint_x;
  static float64 lastpoint_y;
  static float64 lastpoint_yaw_angle;

  //int32 stop_nearest_pointID = FindSTOPNearestPoint();
  gnss_updated = gnss_updated_;
  near_point_id = id_nearest_point;
  float32 cycle_t = 0.02 ;
  bool destination_updated = false;
  bool station_stop=station_stop_;
  if(gnss_updated)
  {
     preview_x_rear = x_rear ;
     preview_y_rear = y_rear ;
     preview_speed = speed ;
     if( preview_speed >3)
     {
       preview_speed =3;
     }
  }
  else
  {
     preview_x_rear = preview_x_rear + (preview_speed*cycle_t+0.5*acceleration*cycle_t*cycle_t)*cos(yaw_angle);
     preview_y_rear = preview_y_rear + (preview_speed*cycle_t+0.5*acceleration*cycle_t*cycle_t)*sin(yaw_angle);
     preview_speed = speed + acceleration*cycle_t;
  }
  

  if(way_point.size()>5)
  {
    // if( lastpoint_x != way_point.back().x)
    // {
    //    lastpoint_x = way_point.back().x;
    //    destination_updated = true;
    // }
    lastpoint_x = way_point.back().x;
    lastpoint_y = way_point.back().y;
    lastpoint_yaw_angle = way_point.back().yaw_angle;
  }

  int32 stop_nearest_pointID = FindSTOPNearestPoint();

  bool is_destination = false;                  // It's not arrive destination.
  // float64 end_point_distance = way_point.back().s-way_point[id_nearest_point].s;   // Calculate the distance from the nearest point  to the end point.
  error = +(x_rear-lastpoint_x)*cos(lastpoint_yaw_angle) + (y_rear-lastpoint_y)*sin(lastpoint_yaw_angle);
  //distance_endpoint = sqrt(pow(x_rear-way_point.back().x,2) + pow(y_rear-way_point.back().y,2));  // Calculate the distance from current vehicle location to the end point.
  if(stop_nearest_pointID >= 0&&stop_nearest_pointID<=way_point.size()-1)
  {
    preview_end_point_distance =  way_point.back().s - way_point[stop_nearest_pointID].s;
  }
  else
  {
    preview_end_point_distance = 0.001 ;
  }
  
  preview_error = +(preview_x_rear-lastpoint_x)*cos(lastpoint_yaw_angle) + (preview_y_rear-lastpoint_y)*sin(lastpoint_yaw_angle);
  //float64 preview_distance_endpoint = sqrt(pow(preview_x_rear-way_point.back().x,2) + pow(preview_y_rear-way_point.back().y,2));  
  
  // std::cout << "\033[32m[error]定点停车误差 : \033[0m"<< error<< std::endl;
  // std::cout << "\033[32m[preview_error]外推后定点停车误差 : \033[0m"<< preview_error<< std::endl;

  // float32 d_t =1.5;
  // float64 distance_range = speed*d_t-1;
  // if (distance_range<1)
  // {
  //   distance_range =1;
  // }
  // else if (distance_range>6)
  // {
  //    distance_range =6;
  // }
  
  bool destination_inrange = false;  //for adas
  if(preview_end_point_distance<6)
  //if(preview_end_point_distance<kDistanceDestination)   // If the distance from the end is less than the set value. 6m.
  {
    is_destination = true;
    if(preview_end_point_distance<1.5)
    // if(preview_end_point_distance<distance_range)
    {
      destination_inrange = true;
    }
  }
  
  //if(error>0.12 && is_destination == true)   //for circle
  if(error>0.12 && is_destination == true && station_stop == true)        // If the nearest point does not exist.
  {
    throttle_value = 0;
    brake_value =kBrakeEmergency;
    drive_flag = BRAKE;
    std::cout << "\033[32m[emergency_brake_value] : \033[0m"<< brake_value << std::endl;
    return 0;
  }
  else if (error>1.5 && is_destination == true && station_stop == false)        // If the nearest point does not exist.
  {
    throttle_value = 0;
    brake_value =kBrakeEmergency;
    drive_flag = BRAKE;
    std::cout << "\033[32m[emergency_brake_value] : \033[0m"<< brake_value << std::endl;
    return 0;
  }

   std::cout << "vehicle_gear_flag : \033[0m"<< vehicle_gear_flag<< std::endl;
  if (vehicle_gear_flag == 1)    // If vehicle gear = D, flag =1 ,vehicle is forward.
  {
    std::cout << "\033[32m[车辆正在前进]\033[0m" << std::endl;

    speed_desire = PreviewSpeed(id_nearest_point);       // find the of speed point that preview.

   if(false == is_destination)                // track speed of the path(You haven't reached the destination yet. Keep following the track).
    {
      Control(id_nearest_point);
      
    }
  else    // run at the destination of the path.
  {
      // std::cout << "\033[32m[即将到达终点]剩余距离 : \033[0m"<< end_point_distance << std::endl;
      // StopDesignatedPoint(end_point_distance,speed_desire,road_slope);
    if(true == station_stop)
    {
      std::cout << "\033[32m[即将到达终点]剩余距离 : \033[0m"<< preview_end_point_distance << std::endl;
      StopDesignatedPoint(preview_end_point_distance,speed_desire,road_slope,error,destination_updated);
    }
    else
    {
      if(!destination_inrange)
      {
        Control(id_nearest_point);
      }
      else  ///
      {
        brake_desire = brake_desire+0.4;
        if(brake_desire <18.0)
        {
          brake_desire =18.0;
        }
        else if (brake_desire >23.0)
        {
          brake_desire =23.0;
        }
         throttle_desire = 0;
         drive_flag_desire = BRAKE;
      }    
     } 
    }
  }

  else      // backward.
  {
    std::cout << "\033[32m[车辆正在倒车]\033[0m" << std::endl;
    if(is_destination == false)
    {
      // std::cout << "\033[32m[还未到达终点]剩余距离 : \033[0m"<< end_point_distance << std::endl;
      // std::cout << "\033[32m[低速控制]期望速度 : \033[0m"<< speed_desire << std::endl;
      // std::cout << "\033[32m[低速控制]current_speed : \033[0m"<< speed << std::endl;
      // std::cout << "\033[32m[低速控制]nearest_speed : \033[0m"<< way_point[id_nearest_point].velocity << std::endl;
      // std::cout << "\033[32m[低速控制]preview_speed_error : \033[0m"<< way_point[id_nearest_point].velocity-speed_desire << std::endl;
      // std::cout << "\033[32m[低速控制]current_speed_error : \033[0m"<< way_point[id_nearest_point].velocity-speed << std::endl;
      LowSpeedControl_throttle(kBackwardSpeed,
                      fabs(speed), 
                      acceleration, 
                      brake_desire, 
                      throttle_desire, 
                      drive_flag_desire);
    }
    else     // run at the destination of the path.
    {
      // std::cout << "\033[32m[即将到达终点]剩余距离 : \033[0m"<< end_point_distance << std::endl;
      // StopDesignatedPoint(end_point_distance,kBackwardSpeed,road_slope);
      //没有考虑倒车时加速度正负的问题  理论上速度为负数 加速度为正；先不管。
      std::cout << "\033[32m[即将到达终点]剩余距离 : \033[0m"<< preview_end_point_distance << std::endl;
      StopDesignatedPoint(preview_end_point_distance,kBackwardSpeed,road_slope,error,destination_updated);
    }
  }
  throttle_value = throttle_desire;
  brake_value = brake_desire;
  drive_flag = drive_flag_desire;
}

// func:    find the id of the nearest point.
// return : return the nearest point id.
int32 longitude_control::LongitudeController::FindNearestPoint()
{
  float64 distance_min = DBL_MAX;
  int32 temp_id = way_point.size()-1;
  // std::cout <<" temp id ="<<temp_id<< std::endl;
  for(int i = 0;i < way_point.size();i++)
  {
    float64 temp_x = way_point[i].x;
    float64 temp_y = way_point[i].y;
    float64 temp_angle = way_point[i].yaw_angle;
    float64 temp_distance = pow((temp_x-x_rear),2)+pow((temp_y-y_rear),2);
    if((temp_distance < distance_min) &&
      (fabs(yaw_angle-temp_angle)<0.5*M_PI||fabs(yaw_angle-temp_angle)>1.5*M_PI))
    {
      distance_min = temp_distance;
      temp_id = i;
    }
  }
  return temp_id;
}

// func:    find the id of the nearest point.
// return : return the nearest point id.
int32 longitude_control::LongitudeController::FindSTOPNearestPoint()
{
  float64 distance_min = DBL_MAX;
  int32 temp_id = way_point.size()-1;
  for(int i = 0;i < way_point.size();i++)
  {
    float64 temp_x = way_point[i].x;
    float64 temp_y = way_point[i].y;
    float64 temp_angle = way_point[i].yaw_angle;
    float64 temp_distance = pow((temp_x-preview_x_rear),2)+pow((temp_y-preview_y_rear),2);
    if((temp_distance < distance_min) &&
      (fabs(yaw_angle-temp_angle)<0.5*M_PI||fabs(yaw_angle-temp_angle)>1.5*M_PI))
    {
      distance_min = temp_distance;
      temp_id = i;
    }
  }
  return temp_id;
}

// func:    find the id of the nearest point for slope.
// return : return the id of the nearest point for slope.
int32 longitude_control::LongitudeController::FindNearestPoint4slope()
{
  float64 distance_min = DBL_MAX;
  int32 temp_id = way_point.size()-1;
  for(int i = 0;i < way_point.size();i++)
  {
    float64 temp_x = way_point[i].x;
    float64 temp_y = way_point[i].y;
    float64 temp_angle = way_point[i].yaw_angle;
    float64 temp_distance = pow((temp_x-(x_rear+3*cos(yaw_angle))),2)+pow((temp_y-(y_rear+3*sin(yaw_angle))),2);
    if((temp_distance < distance_min) &&
      (fabs(yaw_angle-temp_angle)<0.5*M_PI||fabs(yaw_angle-temp_angle)>1.5*M_PI))
    {
      distance_min = temp_distance;
      temp_id = i;
    }
  }
  return temp_id;
}
// find preview speed
/*
float64 longitude_control::LongitudeController::FindPreviewSpeed(const int32 id_nearest_point)
{
  int32 preview_num = 0;
  if(speed>=0)
  {
    preview_num = kForwardPreviewNum;
  }
  else
  {
    preview_num = kBackwardPreviewNum;
  }
  int32 temp_id = id_nearest_point+preview_num<way_point.size()?(id_nearest_point+preview_num):way_point.size()-1;
  float64 speed_desire = way_point[temp_id].velocity;
  return speed_desire;
}
*/

// func:   preview speed accroding the vehicle speed;
// return: return the preview speed.
float64 longitude_control::LongitudeController::PreviewSpeed(const int32 id_nearest_point)
{
  float32 preview_distance = 0;
  const float32 delta_t =1;
  float32 preview_distance_desire = speed* delta_t>2.5?speed*delta_t:2.5;
  for(int32 i = id_nearest_point+1 ; i <way_point.size() ;i++)
  {
    preview_distance += sqrt(pow(way_point[i].x-way_point[i-1].x,2)+pow(way_point[i].y-way_point[i-1].y,2));
   
    if(preview_distance > preview_distance_desire)
    {
      float64 speed_desire = way_point.at(i).velocity;
      return speed_desire;
    }
  }
  return way_point.back().velocity;
}


// func:   update the control mode (decide drive or brake).
void longitude_control::LongitudeController::UpdateDriveFlag(const float64 speed_desire)
{
  float64 speed_error_now = speed - speed_desire;
  if(drive_flag_desire == DRIVE)   // when current state is throttle control.
  {
    // calculate the desired throttle angle.
	float64 DrivingForce_desire = CalDrivingForceDesire(speed_desire, road_slope);   //calculate desired driving force.
	// calculate the desired brake pressure
	float64 BrakingForce_desire = CalBrakingForceDesire(speed_desire, road_slope);  //calculate desired braking force.
    //std::cout << "DrivingForce_desire                        "<< DrivingForce_desire << std::endl;
	//std::cout << "BrakingForce_desire                        "<< BrakingForce_desire << std::endl;
    if(DrivingForce_desire > 0 )   // if the desired throttle angle is positive, the state remains throttle control.
	//if (DrivingForce_desire > 0 && BrakingForce_desire < 0)
    {
        drive_flag_desire = DRIVE;
    }
    else    // if the desired throttle angle is negative.
    {
      if(DrivingForce_desire < 50 && speed_error_now > 0.2) 
	  //if (DrivingForce_desire < 0 && BrakingForce_desire > 0 && speed_error_now > 0.3)
      {
          drive_flag_desire = BRAKE;
      }
      else                           
      {
          drive_flag_desire = DRIVE;
      }
    }
  }
  else if(drive_flag_desire == BRAKE)  // when current state is brake control
  {
	  // calculate the desired throttle angle.
	  float64 DrivingForce_desire = CalDrivingForceDesire(speed_desire, road_slope);   //calculate desired driving force.
	  // calculate the desired brake pressure
	  float64 BrakingForce_desire = CalBrakingForceDesire(speed_desire, road_slope);  //calculate desired braking force.
	  //std::cout << "DrivingForce_desire                        " << DrivingForce_desire << std::endl;
	  //std::cout << "BrakingForce_desire                        " << BrakingForce_desire << std::endl;
	  if ( BrakingForce_desire > 0) 
	  //if (DrivingForce_desire < 0 && BrakingForce_desire > 0)
	  {
		  drive_flag_desire = BRAKE;
	  }
	  else    
	  {
		  if (BrakingForce_desire < 50 && speed_error_now < -0.2) 
		  //if (DrivingForce_desire > 0 && BrakingForce_desire < 0 && speed_error_now < -0.3)
		  {
			  drive_flag_desire = DRIVE;
		  }
		  else                            
			  drive_flag_desire = BRAKE;
		  }
	  }
  }


// func: calculate desired driving force.
float64 longitude_control::LongitudeController::CalDrivingForceDesire(float64 speed_desire, float64 road_slope)
{
    float32 m = 2200;
	float32 g = 9.8;
	float32 f = 0.02;
	float32 nd1 = 1.0;
	float32 Cd = 0.5;
	float32 rou = 1.3;
	float32 A = 2.3;
	float32 cx = 0.5 * Cd * rou*A;
	float32 R = 0.24;

	float64 speed_error = speed-speed_desire;
	static float64 speed_desire_last_time = speed_desire;                 //Store the last time data
	static float64 speed_last_time = speed;                              //Store the last time data
	float64 der_speed_desire = (speed_desire - speed_desire_last_time) / kControlPeriod;     // update the derivative. kControlPeriod = 0.02.
	float64 der_speed = (speed - speed_last_time) / kControlPeriod;                        // update the derivative. kControlPeriod = 0.02.

	integral_speed_error = integral_speed_error + (speed - speed_desire) * kControlPeriod;  // update the integral. kControlPeriod = 0.02.

    float32 DrivingForceDesire1 = m * (der_speed_desire - nd1 * speed_error) + (m*g*f + cx * pow(speed,2) + m * g * (road_slope/180 * M_PI)); 
	//float32 DrivingForceDesire1 = m * (der_speed_desire - nd1 * speed_error) + (m*g*f + cx * pow(speed, 2));
	float64 s11 = speed_error + nd1 * integral_speed_error;
	float64	ds11 = der_speed - der_speed_desire + nd1 * speed_error;
	//FuzzyCtr my_FuzzyCtr;
	//float64 kGainTt = 10 + my_FuzzyCtr.Fuzzy_Rule(s11);                                 //fuzzy control
	float64 kGainFt = 3;                                                               //no fuzzy control

  float64 sign_s11 = 0;
	if (s11 > 0)
		sign_s11 = 1;
	else if (s11 == 0)
		sign_s11 = 0;
	else 
    sign_s11 = -1;

	float64 sat_s11 = 0;
  s11 = s11 / 0.5;
	if (s11 > 1)
		sat_s11 = 1;
	else if (s11 < -1)
		sat_s11 = -1;
	else 
    sat_s11 = s11;

  float32 DrivingForceDesire2 = -kGainFt * sign_s11;
	DrivingForceDesire = DrivingForceDesire1 + DrivingForceDesire2;
	
	/*
    if ( speed > 6.0)
    {
       DrivingForceDesire = DrivingForceDesire1 + DrivingForceDesire2 + 900;
    }
    else
    { 
       DrivingForceDesire = DrivingForceDesire1 + DrivingForceDesire2 + 400;
    }
    */

	/*
	const float64 DrivingForceMaxChange = 10;
	static float64 DrivingForceDesire_last_time = DrivingForceDesire;
	if (DrivingForceDesire - DrivingForceDesire_last_time < -DrivingForceMaxChange)
	{
		DrivingForceDesire = DrivingForceDesire_last_time - DrivingForceMaxChange;
	}
	else if (DrivingForceDesire - DrivingForceDesire_last_time > DrivingForceMaxChange)
	{
		DrivingForceDesire = DrivingForceDesire_last_time + DrivingForceMaxChange;
	}
	BrakingForceDesire_last_time = BrakingForceDesire;
	*/

	if (DrivingForceDesire < 0)
	{
		DrivingForceDesire = 0;
	}
	if (DrivingForceDesire > 4000)
	{
		DrivingForceDesire = 4000;
	}
		
	speed_desire_last_time = speed_desire;
	speed_last_time = speed;

	return DrivingForceDesire;
}

// func: calculate desired braking force.
float64 longitude_control::LongitudeController::CalBrakingForceDesire(float64 speed_desire,float64 road_slope)
{
	float32 m = 2200;
	float32 g = 9.8;
	float32 f = 0.02;
	float32 nd2 = 1.0;
	float32 Cd = 0.4;
	float32 rou = 1.29;
	float32 A = 2.2;
	float32 cx = 0.5 * Cd * rou*A;

	float64 speed_error = speed - speed_desire;
	static float64 speed_desire_last_time = speed_desire;                 //Store the last time data
	static float64 speed_last_time = speed;                               //Store the last time data
	float64 der_speed_desire = (speed_desire - speed_desire_last_time) / kControlPeriod;     // update the derivative. kControlPeriod = 0.02.
	float64 der_speed = (speed - speed_last_time) / kControlPeriod;                        // update the derivative. kControlPeriod = 0.02.
	integral_speed_error = integral_speed_error + (speed - speed_desire) * kControlPeriod;  // update the integral. kControlPeriod = 0.02.

	float32 BrakingForceDesire1 = - m * (der_speed_desire - nd2 * speed_error) - (m*g*f + cx * pow(speed, 2) + m * g * (road_slope / 180 * M_PI));
	//float32 BrakingForceDesire1 = - m * (der_speed_desire - nd2 * speed_error) - (m*g*f + cx * pow(speed, 2));
	float64 s21 = speed_error + nd2 * integral_speed_error;
	float64	ds21 = der_speed - der_speed_desire + nd2 * speed_error;        
	//FuzzyCtr my_FuzzyCtr;
	//float64 kGainFb = 5 + my_FuzzyCtr.Fuzzy_Rule(s21);                                 //fuzzy control
	float64 kGainFb = 6;                                                            //no fuzzy control

  float64 sign_s21 = 0;
	if (s21 > 0)
		sign_s21 = 1;
	else if (s21 == 0)
		sign_s21 = 0;
	else 
    sign_s21 = -1;

	float64 sat_s21 = 0;
  s21 = s21 / 0.5;
	if (s21 > 1)
		sat_s21 = 1;
	else if (s21 < -1)
		sat_s21 = -1;
	else
    sat_s21 = s21;
	float32 BrakingForceDesire2 = -kGainFb * sign_s21;
	BrakingForceDesire = BrakingForceDesire1 + BrakingForceDesire2;

	/*
	const float64 BrakingForceMaxChange = 10;
	static float64 BrakingForceDesire_last_time = BrakingForceDesire;
	if (BrakingForceDesire - BrakingForceDesire_last_time < -BrakingForceMaxChange)
	{
		BrakingForceDesire = BrakingForceDesire_last_time - BrakingForceMaxChange;
	}
	else if (BrakingForceDesire - BrakingForceDesire_last_time > BrakingForceMaxChange)
	{
		BrakingForceDesire = BrakingForceDesire_last_time + BrakingForceMaxChange;
	}
	BrakingForceDesire_last_time = BrakingForceDesire;
	*/

	if (BrakingForceDesire < 0)
	{
		BrakingForceDesire = 0;
	}
	if (BrakingForceDesire > 14000)
	{
		BrakingForceDesire = 14000;
	}

	speed_desire_last_time = speed_desire;
	speed_last_time = speed;

	return BrakingForceDesire;
}

// func:  calculate desired throttle angle.
void longitude_control::LongitudeController::CalThrottleDesire(float64 speed_desire, float64 road_slope)
{ 
  float64 DrivingForce_desire = CalDrivingForceDesire(speed_desire, road_slope);   //calculate desired driving force.
  Interp my_interp;
  throttle_desire = my_interp.Interp2Thr(speed*3.6 , DrivingForce_desire);            // calculate desired throttle angle.
  std::cout << "\033[32m  !!!!DravingForce_desire : \033[0m" << DrivingForce_desire << std::endl;
  std::cout << "\033[32m  !!!!desire_throttle : \033[0m" << throttle_desire << std::endl;
  static float64 throttle_desire_last_time = throttle_desire;
  if(throttle_desire - throttle_desire_last_time < -kThottleMaxChange)
  {
      throttle_desire = throttle_desire_last_time - kThottleMaxChange;
  }
  else if(throttle_desire - throttle_desire_last_time > kThottleMaxChange)
  {
      throttle_desire = throttle_desire_last_time + kThottleMaxChange;
  }
  if(throttle_desire < 0)
  {
      throttle_desire = 0;
  }
  brake_desire = 0;
  throttle_desire_last_time = throttle_desire;
}

// func:  calculate desired brake pressure/angle.
void longitude_control::LongitudeController::CalBrakeDesire(float64 speed_desire, float64 road_slope)
{
  float64 BrakingForce_desire = CalBrakingForceDesire(speed_desire, road_slope);  //calculate desired braking force.
  Interp my_interp;
  brake_desire = my_interp.Interp2Brk(BrakingForce_desire) + 3;            // calculate desired brake pressure.
  //std::cout<<" (acceleration - acce_desire)"<<(acceleration - acce_desire)<<std::endl;
  //std::cout<<"brake_desire"<< brake_desire <<std::endl;
  static float64 brake_desire_last_time = brake_desire;
  std::cout << "\033[32m  !!!!BrakingForce_desire : \033[0m" << BrakingForce_desire << std::endl;
  std::cout << "\033[32m  !!!!brake_desire : \033[0m" << brake_desire << std::endl;
  if(brake_desire - brake_desire_last_time < -kBrakeMaxChange)
  {
    brake_desire = brake_desire_last_time - kBrakeMaxChange;
  }
  else if(brake_desire - brake_desire_last_time > kBrakeMaxChange)
  {
    brake_desire = brake_desire_last_time + kBrakeMaxChange;
  }
  if(brake_desire < 0)
  {
    brake_desire = 0;
  }
  throttle_desire = 0;
  brake_desire_last_time = brake_desire;
}

// func:  stop at designate point.
void longitude_control::LongitudeController::StopDesignatedPoint(const float64 end_point_distance, const float64 speed_desire, const float64 road_slope,const float64 error,const bool destination_updated_)
{
  float64 g = 9.8;
  float64 temp_decel = 0.0;
  float64 revise_decel = 0.0;
  float64 real_decel = 0.0;
  float64 stop_delta_speed = 0.0;
  float64 stop_desire_speed = 0.0;
  float64 temp_brake_desire =0.0;
  static float64 integral_spd = 0.0; 
  // if(destination_updated_)
  // {
  //   integral_temp_spd =0;
  // } 
  std::cout << "\033[32m -error \033[0m" << -error << std::endl;
  if(-preview_error<0.3)  // the distance from the end point is lower than 0.1m , Emergency braking.
  {
    
    if(-error>0.15)  //-0.3~0.15
    {
    brake_desire = brake_desire+0.5;      // plan C
    throttle_desire = 0;
    drive_flag_desire = BRAKE;
    }
    else
    {
    brake_desire = brake_desire+0.5;      // plan C
    throttle_desire = 0;
    drive_flag_desire = BRAKE;
    }
    if(brake_desire>23)
    {
    brake_desire = 23;      // plan C
    throttle_desire = 0;
    drive_flag_desire = BRAKE;
    }
    if(brake_desire<18)
    {
      brake_desire = 18;
    }
    // brake_desire = 40;     
    // throttle_desire = 0;
    // drive_flag_desire = BRAKE;
    integral_temp_spd = 0;
    std::cout << "\033[32m test\033[0m" << std::endl;
    std::cout << "\033[32m nearly stop!!! \033[0m" << std::endl;
   
  }
  //if (fabs(road_slope)<3.5)
  else if (fabs(road_slope)<355)  //-error>0.3
  {
     if (end_point_distance > kDistanceDestination*0.8)  //6m.
    {
        stop_desire_speed = 1;
    }
    else if (end_point_distance > kDistanceDestination*0.5)
    {
        stop_desire_speed = 0.8;
    }
    else if (end_point_distance > kDistanceDestination*0.3)
    {
        stop_desire_speed = 0.7;
    }
    else
    {
        stop_desire_speed = 0.5;
    }
    //stop_desire_speed = 0.4;
    stop_delta_speed = fabs(speed)-stop_desire_speed ;  // speed error.
    if(integral_temp_spd<60 ||stop_delta_speed<0)
    {
      integral_temp_spd = integral_temp_spd + stop_delta_speed * kControlPeriod*5;
    }
    temp_brake_desire = 5 * stop_delta_speed + 0.3 * integral_temp_spd +13; 
    brake_desire =temp_brake_desire;
    std::cout << "\033[32m -5 * stop_delta_speed!!! \033[0m" <<5 * stop_delta_speed<< std::endl;
    std::cout << "\033[32m - 0.3 * integral_temp_spd!!! \033[0m" << 0.3 * integral_temp_spd<< std::endl;      
    drive_flag_desire = BRAKE;
    throttle_desire=0;
  }


  else  // add throttle.
  { 
    std::cout << "\033[32m stop add throttle \033[0m" << std::endl;
    if (end_point_distance > kDistanceDestination*0.8)  //4m.
    {
        stop_desire_speed = 1.2;
    }
    else if (end_point_distance > kDistanceDestination*0.5)
    {
        stop_desire_speed = 0.8;
    }
    else if (end_point_distance > kDistanceDestination*0.3)
    {
        stop_desire_speed = 0.4;
    }
    else
    {
        stop_desire_speed = 0;
    }
    stop_delta_speed = stop_desire_speed - fabs(speed);  // speed error.
    temp_decel = - speed * speed / end_point_distance / 2 + 0.4*g*sin(road_slope*M_PI/180);
    real_decel = - speed * speed / end_point_distance / 2;
    revise_decel = - speed * speed / end_point_distance / 2 + 0.4*g*sin(6 *M_PI/180);
    
    if ((temp_decel>revise_decel)&&(temp_decel>0))   // 当前坡度计算所需的减速度绝对值 < 坡度为4计算出的减速度绝对值，即坡度没达到4deg且当前期望减速度>0需加速.
    {
      if (drive_flag_desire == BRAKE)
      {
         integral_spd = 0.0;
      }
      integral_spd = integral_spd + stop_delta_speed * kControlPeriod;
      throttle_desire = 7 * stop_delta_speed + 0.2 * integral_spd + 1.0 * road_slope ;
      drive_flag_desire = DRIVE;
      brake_desire = 0;
    }

    else if ((temp_decel>revise_decel)&&(temp_decel<0))  // 当前坡度计算所需的减速度绝对值 < 坡度为4计算出的减速度绝对值，即坡度没达到4deg且当前期望减速度<0仍需减速.
    {
      if (drive_flag_desire == DRIVE) 
      {
        integral_spd = 0.0;
      }
      integral_spd = integral_spd + stop_delta_speed * kControlPeriod;
      brake_desire = -5 * stop_delta_speed - 0.2 * integral_spd - 1.0 * road_slope;       
      brake_desire = brake_desire + kStopDeltaBrake ;  // 0.28
      drive_flag_desire = BRAKE;
      throttle_desire=0;
    }

    else if ((temp_decel<=revise_decel)&&(real_decel > -0.1))  // 当前坡度计算所需的减速度绝对值 > 坡度为4计算出的减速度绝对值，即坡度达到4deg且无坡度的期望减速度绝对值<0.01.
    {
      if (drive_flag_desire == BRAKE)
      {
        integral_spd = 0.0;
      }
      integral_spd = integral_spd + stop_delta_speed * kControlPeriod;
      throttle_desire = 7 * stop_delta_speed + 0.2 * integral_spd + 1.0 * road_slope ;
      drive_flag_desire = DRIVE;
      brake_desire = 0;
    }

    else if ((temp_decel<=revise_decel)&&(real_decel <= -0.1) && (real_decel <= acceleration))  // ? 当前坡度计算所需的减速度绝对值 > 坡度为4计算出的减速度绝对值，即坡度达到4deg,无坡度的期望减速度绝对值>0.1,且在当前减速度下仍需要减速.
    {
      if (drive_flag_desire == DRIVE)
      {
        integral_spd = 0.0;
      }
      integral_spd = integral_spd + stop_delta_speed * kControlPeriod;
      brake_desire = -5 * stop_delta_speed - integral_spd * 0.2 - 1.0 * road_slope ;
      brake_desire = brake_desire + kStopDeltaBrake ; 
      drive_flag_desire = BRAKE;
      throttle_desire=0;
      //std::cout<<"ENTER STOP BRAKE 2  "<<std::endl;
    }

    else
    {
      if (brake_desire < 5)
      {
         brake_desire = 5;
      }
      brake_desire = brake_desire - kStopDeltaBrake; // 0.28
      drive_flag_desire = BRAKE;
      throttle_desire=0;
    }
    brake_desire = brake_desire >= 0 ? brake_desire : 0;
    throttle_desire = throttle_desire >= 0 ? throttle_desire : 0;
  }

}

// func:  lower speed control.
void longitude_control::LongitudeController::LowSpeedControl_throttle(const float64 speed_desire_,  // speed desire.
                                          const float64 speed_,            // actual speed.
                                          const float64 acceleration_,     // actual acceleration.
                                          float64 &brake_desire_,          // output: desired brake .
                                          float64 &throttle_desire_,       // output: desired throttle.
                                          DriveFlag &drive_flag_desire_)   // output: desired drive_flag.
{
  float64 tem_ver;
  static float64 temp_desire_brake  = 0.0;
  static float64 temp_desire_throttle = 0.0;
  float32 temp_detal_brake;
  float32 temp_detal_throttle; 
  float32 kDeltathrottle=0.2;
  
  float64 slope_add = 0;
  //if (fabs(road_slope)<2)
  if (fabs(road_slope)<2000)
  {
     slope_add = 0;
  }
  else
  {
    slope_add = road_slope*1.0;
  }

  if(vehicle_gear_flag == 1)
  {
    temp_detal_brake = 5.0;
    //temp_detal_throttle= 7.0;
    temp_detal_throttle= 7.0;            
  }
  else
  {
    temp_detal_brake = 3.0;
    temp_detal_throttle=5.0; 
  }
  //add high_low_control_flag   let integral be zero
  if(high_low_control_flag)
  {
    temp_desire_throttle = 0;
    temp_desire_brake = 0;
    high_low_control_flag = 0;
  }
  //if(speed_desire_-speed_>0.3)   // real speed is lower than desired speed.
  if(speed_desire_-speed_>0.2)   // real speed is lower than desired speed.
  {
        temp_desire_brake = 0;
        temp_desire_throttle = temp_desire_throttle + kDeltathrottle*(speed_desire_ -  fabs(speed_));
        throttle_desire_ = temp_detal_throttle*(speed_desire_ -  fabs(speed_)) + 0.2*temp_desire_throttle +slope_add;//shangpo quexiaopo
        brake_desire_=0;
        // std::cout << "\033[32m temp_desire_throttle : \033[0m" << temp_desire_throttle << std::endl;
        // std::cout << "\033[32m temp_detal_throttle*(speed_desire_ -  fabs(speed_)) : \033[0m" << temp_detal_throttle*(speed_desire_ -  fabs(speed_)) << std::endl;
        // std::cout << "\033[32m  0.2*temp_desire_throttle : \033[0m" <<  0.2*temp_desire_throttle << std::endl;
       
  }
  //else if (speed_desire_-speed_<-0.3)                         // real speed is larger than desired speed.
  else if (speed_desire_-speed_<-0.2)                         // real speed is larger than desired speed.
  {
        temp_desire_throttle = 0;
        temp_desire_brake = temp_desire_brake + kDeltaBrake*(fabs(speed_) - speed_desire_);
      //  std::cout<<"temp_desire_brake = "<<temp_desire_brake<<std::endl;
      //  brake_desire_ = 0.2*temp_desire_brake + temp_detal_brake*(speed_ - speed_desire_)-slope_add;//shangpo quexiaopo
      brake_desire_ =  temp_detal_brake*(speed_ - speed_desire_)+0.2*temp_desire_brake - slope_add;
        // brake_desire_ =  temp_detal_brake*(fabs(speed_) - speed_desire_);
        throttle_desire_=0;
  }
  else
  {
    brake_desire_ = brake_desire_>=0?brake_desire_:0;
    throttle_desire_ = throttle_desire_>=0?throttle_desire_:0;
  }

  brake_desire_ = brake_desire_>0?brake_desire_:0;
  throttle_desire_ = throttle_desire_>0?throttle_desire_:0;
  if (brake_desire_>0)
  {
     drive_flag_desire_ = BRAKE; 
  }
  else if (throttle_desire_>0)
  {
     drive_flag_desire_ = DRIVE;
  }
  std::cout << "\033[32m temp_desire_throttle : \033[0m" << temp_desire_throttle << std::endl;
  std::cout << "\033[32m temp_detal_throttle*(speed_desire_ -  fabs(speed_)) : \033[0m" << temp_detal_throttle*(speed_desire_ -  fabs(speed_)) << std::endl;
  std::cout << "\033[32m  0.2*temp_desire_throttle : \033[0m" <<  0.2*temp_desire_throttle << std::endl;

  std::cout << "\033[32m temp_desire_brake : \033[0m" << temp_desire_brake << std::endl;
  std::cout << "\033[32m temp_detal_brake*(speed_ - speed_desire_) : \033[0m" << temp_detal_brake*(speed_ - speed_desire_) << std::endl;
  std::cout << "\033[32m 0.2*temp_desire_brake : \033[0m" <<  0.2*temp_desire_brake << std::endl;

  std::cout << "\033[32m throttle_desire_ : \033[0m"<< throttle_desire_ << std::endl;
  std::cout << "\033[32m brake_desire_ : \033[0m"<< brake_desire_ << std::endl;
}

// func:   calculate current road slope;
// return: return the road slope.
float64 longitude_control::LongitudeController::CalCulateSlope()
{
    //  当决策发的路点有Z信息时用这个
	/*
    static float64 last_x_rear = x_rear;               // rear axle
    static float64 last_y_rear = y_rear;               // rear axle 
    static float64 last_z_rear = z_rear;
    static float64 road_slope = 0;

    static std::vector<float64> slope_vector = {road_slope, road_slope, road_slope, road_slope, road_slope};
    float64 delta_x4slope = x_rear - last_x_rear;
    float64 delta_y4slope = y_rear - last_y_rear;
    float64 delta_z4slope = z_rear - last_z_rear;
    if(delta_x4slope*delta_x4slope+delta_y4slope*delta_y4slope>1)
    {
      road_slope = (asin(delta_z4slope/sqrt(pow(delta_x4slope,2)+pow(delta_y4slope,2))))/M_PI*180;
      road_slope = fmod(road_slope+180,360)-180;
      road_slope = 0.3*road_slope + 0.7*slope_vector[0];
      slope_vector.insert(slope_vector.begin(), road_slope);
      slope_vector.pop_back();
      last_x_rear = x_rear;               // rear axle
      last_y_rear = y_rear;               // rear axle 
      last_z_rear = z_rear;
    }
    else
    {
      road_slope = road_slope;
    }
  */

	//运动学卡尔曼滤波
	Kalman my_kalman;
	road_slope = my_kalman.KalmanFilter(speed, acceleration);

  return road_slope;
}

// func:  lower speed control.
void longitude_control::LongitudeController::LowSpeedControl(const float64 speed_desire_,  // speed desire.
                                          const float64 speed_,            // actual speed.
                                          const float64 acceleration_,     // actual acceleration.
                                          float64 &brake_desire_,          // output: desired brake .
                                          float64 &throttle_desire_,       // output: desired throttle.
                                          DriveFlag &drive_flag_desire_)   // output: desired drive_flag.
{
  // float64 tem_ver;
  // if(vehicle_gear_flag != 1)
  // {
  //   tem_ver = 0.1;
  // }
  // else
  // {
  //   tem_ver = 0;
  // }

  if(speed_desire_>speed_)   // real speed is lower than desired speed.
  {
    if(acceleration_> 0.1)    // if accelerating now.
    {
      brake_desire_ = brake_desire_;
    }
    else                     // if decelerating now.
    {
      brake_desire_ = brake_desire_ - kDeltaBrake;   //kDeltaBrake = 0.15
    }

  }
  else                          // real speed is larger than desired speed.
  {
    if(acceleration_>-0.1)      // if accelerating now.
    {
      brake_desire_ = brake_desire_ + kDeltaBrake;
    }
    else                        // if decelerating now.
    {
      brake_desire_ = brake_desire_;
    }
  }
  brake_desire_ = brake_desire_>=0?brake_desire_:0;
  throttle_desire_ = 0;
  drive_flag_desire_ = BRAKE;
}

// func:  emergency brake.
void longitude_control::LongitudeController::EmergencyBrake(float64 &throttle_value,
                                             float64 &brake_value,
                                             DriveFlag &drive_flag)
{
  throttle_value = 0;
  brake_value = kBrakeEmergency;   // Brake pressure may have to be reduced.
  drive_flag = BRAKE;
}



void longitude_control::LongitudeController::ShiftGearBrake(float64 &throttle_value,
                                                            float64 &brake_value,
                                                            DriveFlag &drive_flag)
{
  throttle_value = 0;
  brake_value = kBrakeShiftGear;
  drive_flag = BRAKE;
}

void longitude_control::LongitudeController::Control(int32 id_nearest_point)
{
      integral_temp_spd =0;
      std::cout << "\033[32m[还未到达终点]剩余距离 : \033[0m"<< preview_end_point_distance << std::endl;
      if(speed < kCriticalSpeed && acceleration<1)           // desired speed is too small and acceleration is small, use lowspeed control. kCriticalSpeed = 2.2m/s 
      //  if(speed_desire<kCriticalSpeed && acceleration<1)     
      {
        std::cout << "\033[32m[低速控制]期望速度 : \033[0m"<< speed_desire << std::endl;
        std::cout << "\033[32m[低速控制]当前速度 : \033[0m"<< speed << std::endl;
        //std::cout << "\033[32m[低速控制]最近点速度 : \033[0m"<< way_point[id_nearest_point].velocity << std::endl;
        //std::cout << "\033[32m[低速控制]当前速度误差 : \033[0m"<< way_point[id_nearest_point].velocity-speed << std::endl;
        //std::cout << "\033[32m[低速控制]预瞄速度误差 : \033[0m"<< speed_desire-speed << std::endl;
        LowSpeedControl_throttle(speed_desire,
                        speed, 
                        acceleration, 
                        brake_desire, 
                        throttle_desire, 
                        drive_flag_desire);
      }
      else  // if the speed of vehicle is large.
      {
        std::cout << "\033[32m[高速控制]期望速度 : \033[0m"<< speed_desire << std::endl;
        std::cout << "\033[32m[高速控制]当前速度 : \033[0m"<< speed << std::endl;
        std::cout << "\033[32m[高速控制]最近点速度 : \033[0m"<< way_point[id_nearest_point].velocity<< std::endl;
        std::cout << "\033[32m[高速控制]当前速度误差 : \033[0m"<< way_point[id_nearest_point].velocity - speed<< std::endl;
        //std::cout << "\033[32m[高速控制]预瞄速度误差 : \033[0m"<< speed-speed_desire << std::endl;
        UpdateDriveFlag(speed_desire);           // Calculate drive mode according to acceleration.
        high_low_control_flag = 1;              // if the speed of vehicle is large.
        if(drive_flag_desire == DRIVE)          // if vehicle need increase the speed.
        { 
          // std::cout << "\033[32m[高速控制]throttle!!!!! : \033[0m" << std::endl;
          CalThrottleDesire(speed_desire, road_slope);       // throttle control.
        }
        else if(drive_flag_desire == BRAKE)     // if vehicle need reduce the speed.
        {
          CalBrakeDesire(speed_desire, road_slope);          // brake control.
        }
      }
}