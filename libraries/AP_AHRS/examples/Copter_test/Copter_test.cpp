
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_ADC/AP_ADC.h>
#include <AP_ADC_AnalogSource/AP_ADC_AnalogSource.h>
#include <AP_Baro/AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Baro/AP_Baro.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <Filter/Filter.h>
#include <SITL/SITL.h>
#include <AP_Buffer/AP_Buffer.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <DataFlash/DataFlash.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_Rally/AP_Rally.h>
#include <AP_Scheduler/AP_Scheduler.h>

#include <AP_HAL_AVR/AP_HAL_AVR.h>
#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_Empty/AP_HAL_Empty.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_HAL_PX4/AP_HAL_PX4.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

#include <AC_PID/AC_PID.h>

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

// INS and Baro declaration
AP_InertialSensor ins;
Compass compass;
AP_GPS gps;
AP_Baro baro;
AP_SerialManager serial_manager;
AP_AHRS_DCM  ahrs(ins, baro, gps);

// Motor numbers definitions
#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right
#define RC_THR_MIN 1100

// PID array (6 pids, two for each axis)
AC_PID PID_PITCH_RATE(0.7, 1, 0, 50, 20, 0);
AC_PID PID_ROLL_RATE(0.7, 1, 0, 50, 20, 0);
AC_PID PID_YAW_RATE(1.4, 1, 0, 50, 20, 0);

AC_PID PID_PITCH_STAB(4.5, 0, 0, 100, 0, 0);
AC_PID PID_ROLL_STAB(4.5, 0, 0, 100, 0, 0);
AC_PID PID_YAW_STAB(4.5, 0, 0, 100, 0, 0);


void setup() {
  hal.console->println("Starting AP_HAL::RCOutput test");
  for (uint8_t i=0; i < 8; i++) {
      hal.rcout->enable_ch(i);
  }
  
  ins.init(AP_InertialSensor::COLD_START, 
      AP_InertialSensor::RATE_100HZ);
  ahrs.init();
  serial_manager.init();

  if( compass.init() ) {
      hal.console->printf("Enabling compass\n");
      ahrs.set_compass(&compass);
  } else {
      hal.console->printf("No compass detected\n");
  }
  gps.init(NULL, serial_manager);
}

void loop() {
  static float yaw_target = 0;
  static int16_t channels[8] = { 0 }; 
  uint32_t iTimeStamp = hal.scheduler->millis();
  
  // Wait until new orientation data (normally 5ms max)
  ins.wait_for_sample();
 
  // Read RC channels and store in channels array
  for (uint8_t i=0; i < 8; i++) {
      uint16_t v = hal.rcin->read(i);
      channels[i] = v;
  }

  long rcthr, rcyaw, rcpit, rcrol;  // Variables to store radio in 

  // Read RC transmitter 
  rcrol = channels[0];
  rcpit = channels[1];
  rcthr = channels[2];
  rcyaw = channels[3];
  
  // Ask MPU6050 for orientation
  ahrs.update();
  float roll = ToDeg(ahrs.roll) ;
  float pitch = ToDeg(ahrs.pitch) ;
  float yaw = ToDeg(ahrs.yaw) ;
  
  // Ask MPU6050 for gyro data
  Vector3f gyro = ins.get_gyro();
  float gyroPitch = ToDeg(gyro.y), gyroRoll = ToDeg(gyro.x), gyroYaw = ToDeg(gyro.z);
  
  // Do the magic
  if(rcthr > RC_THR_MIN + 100) {  // Throttle raised, turn on stablisation.
      // Stablise PIDS
      float dT = hal.scheduler->millis() - iTimeStamp;
      PID_PITCH_STAB.set_input_filter_all((float)rcpit - pitch);
      PID_PITCH_STAB.set_dt(dT);
      PID_ROLL_STAB.set_input_filter_all((float)rcrol - roll);
      PID_ROLL_STAB.set_dt(dT);
      PID_YAW_STAB.set_input_filter_all((float)rcyaw - yaw);
      PID_YAW_STAB.set_dt(dT);
      
      float pitch_stab_output = constrain_float(PID_PITCH_STAB.get_pid(), -250, 250); 
      float roll_stab_output = constrain_float(PID_ROLL_STAB.get_pid(), -250, 250);
      float yaw_stab_output = constrain_float(PID_YAW_STAB.get_pid(), -360, 360);
    
      // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
      if(abs(rcyaw ) > 5) {
          yaw_stab_output = rcyaw;
          yaw_target = yaw;   // remember this yaw for when pilot stops
      }
      
      PID_PITCH_RATE.set_input_filter_all((float)pitch_stab_output - gyroPitch);
      PID_PITCH_RATE.set_dt(dT);
      PID_ROLL_RATE.set_input_filter_all((float)roll_stab_output - gyroRoll);
      PID_ROLL_RATE.set_dt(dT);
      PID_YAW_RATE.set_input_filter_all((float)yaw_stab_output - gyroYaw);
      PID_YAW_RATE.set_dt(dT);
      
      // rate PIDS
      long pitch_output =  constrain_float(PID_PITCH_RATE.get_p(), - 500, 500);  
      long roll_output =  constrain_float(PID_ROLL_RATE.get_p(), -500, 500);  
      long yaw_output =  constrain_float(PID_YAW_RATE.get_p(), -500, 500);  

      // mix pid outputs and send to the motors.
      hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
      hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
      hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
      hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);
  } 
  else {
      // motors off
      hal.rcout->write(MOTOR_FL, 1000);
      hal.rcout->write(MOTOR_BL, 1000);
      hal.rcout->write(MOTOR_FR, 1000);
      hal.rcout->write(MOTOR_BR, 1000);
        
      // reset yaw target so we maintain this on takeoff
      yaw_target = yaw;
      
      // reset PID integrals whilst on the ground
      PID_PITCH_STAB.set_integrator(0);
      PID_ROLL_STAB.set_integrator(0);
      PID_YAW_STAB.set_integrator(0);
  }
}

AP_HAL_MAIN();
