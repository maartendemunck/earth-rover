#ifndef __VCU__
#define __VCU__


#include <cstdint>
#include <i2c_t3.h>
#include <GPSport.h>
#include <NMEAGPS.h>
#include "adafruit_bno055.hpp"
#include "configured-servo.hpp"
#include "lighting.hpp"


namespace earth_rover
{

  class Vcu
  {

    ConfiguredServo steering_servo;
    ConfiguredServo throttle_servo;
    ConfiguredServo gearbox_servo;
    Lighting automotive_lighting;
    AdafruitBNO055<i2c_t3> bno055_imu;
    NMEAGPS mtk3339_gps;
    gps_fix mtk3339_gps_fix;

    elapsedMillis since_last_control_message;
    bool timeout_handler_called;

    bool is_driving;
    elapsedMillis since_driving;

    public:

      Vcu(uint8_t steering_servo_pin, uint8_t throttle_servo_pin, uint8_t gearbox_servo_pin,
          uint8_t head_lamp_pin, uint8_t tail_lamp_pin, uint8_t turn_signal_right_pin, uint8_t turn_signal_left_pin);
      ~Vcu() = default;
      void setup();
      void spinOnce();

      void handleControlMessage(int16_t steering, int16_t throttle, int8_t gearbox, int8_t lighting);

      AdafruitBNO055<i2c_t3>::bno055_euler_angles_t getOrientation();
      AdafruitBNO055<i2c_t3>::bno055_calibration_status_t getImuCalibrationStatus();
      gps_fix getGpsData() const { return mtk3339_gps_fix; };

    private:

      void handleTimeout();

  };

}

#endif