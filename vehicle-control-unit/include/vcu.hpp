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
    i2c_t3 & bno055_imu_i2c_device;
    const uint8_t bno055_imu_i2c_scl_pin;
    const uint8_t bno055_imu_i2c_sda_pin;
    AdafruitBNO055<i2c_t3> bno055_imu_device;
    HardwareSerial & mtk3339_gps_serial_device;
    const uint8_t mtk3339_gps_serial_rx_pin;
    const uint8_t mtk3339_gps_serial_tx_pin;
    NMEAGPS mtk3339_gps_device;
    gps_fix mtk3339_gps_fix;

    elapsedMillis since_last_control_message;
    bool timeout_handler_called;

    bool is_driving;
    elapsedMillis since_driving;

    public:

      Vcu(uint8_t steering_servo_pin, uint8_t throttle_servo_pin, uint8_t gearbox_servo_pin,
        uint8_t head_lamp_pin, uint8_t tail_lamp_pin, uint8_t turn_signal_right_pin, uint8_t turn_signal_left_pin,
        i2c_t3 & imu_i2c, uint8_t imu_i2c_scl_pin, uint8_t imu_i2c_sda_pin,
        HardwareSerial & gps_serial, uint8_t gps_serial_rx_pin, uint8_t gps_serial_tx_pin);
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