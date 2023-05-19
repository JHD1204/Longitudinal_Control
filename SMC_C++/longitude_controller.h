/*******************************************************************/
/*                      Author: xubiao                             */
/*                     Contact: xubiao@hnu.edu.cn                  */
/*                 Last update: 2019-08-16                         */
/*******************************************************************/

/*
  Description: The header file for C++ class LongitudeController
  The class includes all the necessary vars and funcs for longitude control.
*/

#ifndef _LONGITUDE_SMC_CONTROLLER_H_
#define _LONGITUDE_SMC_CONTROLLER_H_

#include <vector>

#include "my_typedef.h"

namespace longitude_control{
typedef struct{
    float64 x;                    // GLobal x coordination of the way point
    float64 y;                    // GLobal y coordination of the way point
    float64 z;                    // Global z coordination of the way point
    float32 yaw_angle;            // Yaw angle of the way point
    float32 velocity;             // Target speed of the way point
    float32 curvature;            // Curvature of the way point
    float64 s;                    // distance
    uint16 proprety;
}WayPointStruct;
enum DriveFlag {DRIVE = 1,
                BRAKE = 0};
// enum GearMode_
// {
//   P = 0,
//   R = 1,
//   N = 2,
//   D = 3
// };

//  using vehicle_control::CVehicleControl::GearMode;
class LongitudeController
{
public:
  LongitudeController();
  ~LongitudeController();
  int32 CalLongitueCommand(const std::vector<WayPointStruct> way_point_,  // path.
                           const float64 x0_,                             // vehicle rear axle position x.
                           const float64 y0_,                             // vehicle rear axle position y.
                           const float64 z0_,
                           const float64 yaw_angle_,                      // vehicle position yaw angle, rad.
                           const float64 pitch_angle_,
                           const float64 speed_,                          // vehicle speed, m/s, forward is +, back is -.
                           const float64 acceleration_,                   // vehicle acceleration.
                           const float64 throttle_,                       // throttle angle, deg.
                           const float64 brake_,                          // brake angle/pressure, deg/MPa.
                           const bool vehicle_gear_flag0,
                           const bool gnss_updated_,
                           const bool station_stop_,
                           float64 &throttle_value,                       // throttle angle, deg.
                           float64 &brake_value,                          // brake pressure/angle, MPa/deg.
                           DriveFlag &drive_flag,                         // 1: drive; 0:brake.
                           int32 &near_point_id                   
                           );                         // nearest point id.
  int32 FindNearestPoint();                                   // func: find the id of the nearest point.
  int32 FindSTOPNearestPoint();                               // func: find the id of the nearest point for stop.
  int32 FindNearestPoint4slope();                               // func: find the id of the nearest point for slope.
  float64 PreviewSpeed(const int32 id_nearest_point);         // func: find the preview speed.
  void UpdateDriveFlag(const float64 speed_desire);            // func: decide drive or brake.
  float64 CalCulateSlope();                                                               //// func: calculate slope of road.
  float64 CalDrivingForceDesire(const float64 speed_desire, const float64 road_slope);           // func: calculate desired driving force.
  float64 CalBrakingForceDesire(const float64 speed_desire, const float64 road_slope);           // func: calculate desired braking force.
  void CalThrottleDesire(const float64 speed_desire,const float64 road_slope);                // func: calculate desired throttle angle.
  void CalBrakeDesire(const float64 speed_desire, const float64 road_slope);                   // func: calculate desired brake pressure/angle.
  void StopDesignatedPoint(const float64 end_point_distance,
                           const float64 speed_desire,
                           const float64 road_slope,
                           const float64 error,
                           const bool integral_temp_spd);   // func: stop at designate point.
  void LowSpeedControl_throttle(const float64 speed_desire_,     // speed desire.
                       const float64 speed_,            // actual speed.
                       const float64 acceleration_,     // actual acceleration.
                       float64 &brake_desire_,          // output: desired brake .
                       float64 &throttle_desire_,       // output: desired throttle.
                       DriveFlag &drive_flag_desire_);  // output: desired drive_flag.
  void LowSpeedControl(const float64 speed_desire_,     // speed desire.
                       const float64 speed_,            // actual speed.
                       const float64 acceleration_,     // actual acceleration.
                       float64 &brake_desire_,          // output: desired brake .
                       float64 &throttle_desire_,       // output: desired throttle.
                       DriveFlag &drive_flag_desire_);  // output: desired drive_flag.
  void EmergencyBrake(float64 &throttle_value,          // throttle angle, deg.
                      float64 &brake_value,             // brake pressure/angle, MPa/deg.
                      DriveFlag &drive_flag);           // 1: drive; 0:brake.
  void ShiftGearBrake(float64 &throttle_value,          // throttle angle, deg.
                      float64 &break_value,             // brake pressure/angle, MPa/deg.
                      DriveFlag &drive_flag);
  
  void Control(int32 id_nearest_point);

  inline float64 GetSpeedDesire(){return speed_desire;}
  inline float64 GetDistance_endpoint(){return distance_endpoint;}
  inline float64 GetDrivingForceDesire() { return DrivingForceDesire; }
  inline float64 GetBrakingForceDesire() { return BrakingForceDesire; }

  float64 distance_endpoint;
  float64 preview_end_point_distance;   //for print 
  float64 error;                //for print 
  float64 preview_error;                //for print 
  float64 road_slope;                   //for print 
  bool high_low_control_flag;           //0=low 1=high
  float64 speed_error;

private:
  // control command
  float64 throttle_desire;     // %.
  float64 brake_desire;        // %/Mpa.
  DriveFlag drive_flag_desire;
  float32 DrivingForceDesire;
  float32 BrakingForceDesire;

  // state variable
  std::vector<WayPointStruct> way_point;
  float64 x_front;
  float64 y_front;
  float64 x_rear;
  float64 y_rear;
  float64 z_rear;
  float64 preview_x_rear;
  float64 preview_y_rear;
  float64 preview_speed;
  float64 yaw_angle;
  float64 pitch_angle;
  float64 speed;
  float64 acceleration;
  float64 throttle;
  float64 brake;
  float64 speed_desire;
  float64 acc_desire;

  float64  integral_temp_spd ;   // for StopDesignatedPoint


  // intermediate variable
  float64 integral_speed_error;
  //GearMode vehicle_gear_num;
  bool gnss_updated;
  bool vehicle_gear_flag;   
  //bool high_low_control_flag;  //0=low 1=high
  // parameter
  const int32   kForwardPreviewNum ;
  const int32   kBackwardPreviewNum ;
  const float64 kBrakeEmergency ;
  const float64 kBrakeShiftGear ;
  const float64 kDistanceDestination ;
  const float64 kGainAcce ;
  const float64 kAcceMax ;
  const float64 kAcceMin ;
  const float64 kGainTe ;
  const float64 kGainTb ;
  const float64 kControlPeriod ;
  const float64 kCriticalSpeed ;      // critical speed of acceleration control and speed control
  const float64 kAcceTolerant ;     // tolerant acceleration of switch from drive to brake
  const float64 kDeceTolerant ;     // tolerant deceleration of switch from brake to drive
  const float64 kThottleMaxChange ; // max change value of throttle
  const float64 kBrakeMaxChange ;   // max change value of brake
  const float64 kBrakeMax ;          // max brake command
  const float64 kDeltaBrake;       // brake change value in LowSpeedControl and StopDesignatedPoint
  const float64 kStopDeltaBrake;
  const float64 kBackwardSpeed ;   // desired backward speed, m/s
  
}; // class.
}  // namespace.
#endif
