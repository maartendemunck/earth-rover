#include <Arduino.h>
#include "vcu.hpp"


namespace earth_rover
{

  Vcu::Vcu (uint8_t steering_servo_pin, uint8_t throttle_servo_pin, uint8_t gearbox_servo_pin,
      uint8_t head_lamp_pin, uint8_t tail_lamp_pin, uint8_t turn_signal_right_pin, uint8_t turn_signal_left_pin,
      i2c_t3 & imu_i2c, uint8_t imu_i2c_scl_pin, uint8_t imu_i2c_sda_pin,
      HardwareSerial & gps_serial, uint8_t gps_serial_rx_pin, uint8_t gps_serial_tx_pin):
    steering_servo {steering_servo_pin, 2000u, 1000u, 1500u, true},
    throttle_servo {throttle_servo_pin, 1000u, 2000u, 1500u, true},
    gearbox_servo {gearbox_servo_pin, 1150u, 1850u, 1480u, true},
    automotive_lighting {head_lamp_pin, tail_lamp_pin, turn_signal_right_pin, turn_signal_left_pin},
    bno055_imu_i2c_device {imu_i2c},
    bno055_imu_i2c_scl_pin {imu_i2c_scl_pin},
    bno055_imu_i2c_sda_pin {imu_i2c_sda_pin},
    bno055_imu_device {imu_i2c, AdafruitBNO055<i2c_t3>::BNO055_ADDRESS_A},
    mtk3339_gps_serial_device {gps_serial},
    mtk3339_gps_serial_rx_pin {gps_serial_rx_pin},
    mtk3339_gps_serial_tx_pin {gps_serial_tx_pin}
  {
    ;
  }


  void Vcu::setup()
  {
    // Configure the servos. The steering servo requires an explicit setPosition (on top of the one in the setup
    // function), so send an explicit center position to all servos.
    steering_servo.setup();
    steering_servo.setPosition(0);
    throttle_servo.setup();
    throttle_servo.setPosition(0);
    gearbox_servo.setup();
    gearbox_servo.setPosition(0);
    // Start with stop lamps and hazard flashers on, until we receive something from the HMI.
    automotive_lighting.setStopLamps(true);
    automotive_lighting.setHazardFlashers(true);
    // Initialize the Bosch BNO055 IMU. TODO: get I²C configuration as a parameter?
    bno055_imu_i2c_device.begin(I2C_MASTER, 0x00, bno055_imu_i2c_scl_pin, bno055_imu_i2c_sda_pin, I2C_PULLUP_EXT,
                                400000, I2C_OP_MODE_ISR);
    bno055_imu_device.setup();
    // Initialize the GPS. TODO: get serial configuration as a parameter?
    mtk3339_gps_serial_device.setRX(mtk3339_gps_serial_rx_pin);
    mtk3339_gps_serial_device.setTX(mtk3339_gps_serial_tx_pin);
    mtk3339_gps_serial_device.begin(9600, SERIAL_8N1);
    // This is a safe state, no need to call the timeout handler.
    timeout_handler_called = true;
  }


  void Vcu::spinOnce()
  {
    if(since_last_control_message > 500u && !timeout_handler_called)
    {
      handleTimeout();
    }
    automotive_lighting.spinOnce();
    while(mtk3339_gps_device.available(mtk3339_gps_serial_device))
    {
      mtk3339_gps_fix = mtk3339_gps_device.read();
    }
  }


  void Vcu::handleControlMessage(int16_t steering, int16_t throttle, int8_t gearbox, int8_t lighting)
  {
    // Set steering servo.
    steering_servo.setPosition(steering);
    // Set throttle servo.
    throttle_servo.setPosition(throttle);
    // Set gearbox servo. Prevent shifting to low or high gear while not driving.
    if(!is_driving && abs(throttle) >= 150)
    {
      since_driving = 0;
      is_driving = true;
    }
    else if(abs(throttle) < 100)
    {
      is_driving = false;
    }
    switch(gearbox)
    {
      case 1:  // Low
        if(is_driving && since_driving > 200u)
        {
          gearbox_servo.setPosition(-1000);
        }
        break;
      case 2:  // High
        if(is_driving && since_driving > 200u)
        {
          gearbox_servo.setPosition(1000);
        }
        break;
      default:  // Neutral
        gearbox_servo.setPosition(0);
        break;
    }
    // Set automotive lighting.
    automotive_lighting.setTurnSignalRight(lighting & 0x01);
    automotive_lighting.setTurnSignalLeft(lighting & 0x02);
    automotive_lighting.setDippedBeam(lighting & 0x04);
    automotive_lighting.setHighBeam(lighting & 0x08);
    automotive_lighting.setHazardFlashers(lighting & 0x10);
    automotive_lighting.setStopLamps(throttle > -50 && throttle < 50);
    // Reset control message timout.
    since_last_control_message = 0;
    timeout_handler_called = false;
  }
  

  AdafruitBNO055<i2c_t3>::bno055_euler_angles_t Vcu::getOrientation()
  {
    // Get Euler angles.
    auto angles = bno055_imu_device.getEulerAngles();
    // Adjust Euler angles for the orientation of the Adafruit BNO055 IMU in the car.
    angles.yaw = - angles.yaw - 90.;
    while(angles.yaw < 0)
    {
      angles.yaw += 2. * M_PI;
    }
    angles.pitch = angles.pitch;
    angles.roll = - angles.roll;
    return angles;
  }


  AdafruitBNO055<i2c_t3>::bno055_calibration_status_t Vcu::getImuCalibrationStatus()
  {
    return bno055_imu_device.getCalibrationStatus();
  }


  void Vcu::handleTimeout ()
  {
    // Stop car and turn stop lamps and hazard flashers on.
    throttle_servo.setPosition(0);
    automotive_lighting.setStopLamps(true);
    automotive_lighting.setHazardFlashers(true);
    // Timeout handler called.
    timeout_handler_called = true;
  }

}