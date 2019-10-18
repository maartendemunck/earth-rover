//! Powertrain device driver for the Earth Rover's VCU (implementation).
/*!
 *  Device driver for the Earth Rover's powertrain, controlling the electronic speed controller (ESC) and the gearbox
 *  servo.
 * 
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "powertrain.hpp"


namespace earth_rover_vcu
{

  Powertrain::Powertrain(uint8_t esc_pin_number, uint8_t gearbox_servo_pin_number)
  :
    esc_pin_number {esc_pin_number},
    gearbox_servo_pin_number {gearbox_servo_pin_number},
    esc {esc_pin_number, 1000u, 1500u, 2000u, true},
    gearbox_servo {gearbox_servo_pin_number, 1000u, 1500u, 2000u, false},
    gearbox_pulse_widths {1480u, 1150u, 1800u},
    current_throttle_setting {0},
    current_gear {0}
  {
    ;
  }


  void Powertrain::setup()
  {
    current_throttle_setting = 0;
    esc.setup();
    esc.setPosition(current_throttle_setting);
    current_gear = 0;
    gearbox_servo.setup();
    gearbox_servo.setPulseWidth(gearbox_pulse_widths[current_gear]);
  }


  void Powertrain::spinOnce()
  {
    if(current_gear != requested_gear)
    {
      if(requested_gear == 0 || (is_driving && since_driving > 200u))
      {
        gearbox_servo.setPulseWidth(gearbox_pulse_widths[requested_gear]);
        current_gear = requested_gear;
      }
    }
  }


  void Powertrain::setNormalizedThrottleSetting(int16_t throttle_setting)
  {
    esc.setPosition(throttle_setting);
    current_throttle_setting = throttle_setting;
    if(!is_driving && abs(current_throttle_setting) >= 150)
    {
      is_driving = true;
      since_driving = 0;
    }
    else if(is_driving && abs(current_throttle_setting) <= 100)
    {
      is_driving = false;
    }
  }


  void Powertrain::configureESC(uint16_t pulse_width_reverse, uint16_t pulse_width_stop, uint16_t pulse_width_forward)
  {
    ConfiguredServo::Configuration configuration
      {pulse_width_reverse, pulse_width_stop, pulse_width_forward, pulse_width_stop, true};
    esc.setConfiguration(configuration);
    esc.setPosition(current_throttle_setting);
  }


  void Powertrain::configureGearboxServo(
    uint16_t pulse_width_neutral, uint16_t pulse_width_low, uint16_t pulse_width_high)
  {
    gearbox_pulse_widths[0] = pulse_width_neutral;
    gearbox_pulse_widths[1] = pulse_width_low;
    gearbox_pulse_widths[2] = pulse_width_high;
    gearbox_servo.setPulseWidth(gearbox_pulse_widths[current_gear]);
  }


  void Powertrain::configureGearboxServo(int8_t gear, uint16_t pulse_width)
  {
    if(gear >= 0 && gear <= 2)
    {
      gearbox_pulse_widths[gear] = pulse_width;
      if(gear == current_gear)
      {
        gearbox_servo.setPulseWidth(gearbox_pulse_widths[current_gear]);
      }
    }
  }


  Powertrain::Configuration Powertrain::getConfiguration()
  {
    Configuration configuration;
    auto esc_configuration = esc.getConfiguration();
    configuration.esc.pulse_width_reverse = esc_configuration.minimum_pulse_width;
    configuration.esc.pulse_width_stop = esc_configuration.center_pulse_width;
    configuration.esc.pulse_width_forward = esc_configuration.maximum_pulse_width;
    configuration.gearbox.pulse_width_neutral = gearbox_pulse_widths[0];
    configuration.gearbox.pulse_width_low = gearbox_pulse_widths[1];
    configuration.gearbox.pulse_width_high = gearbox_pulse_widths[2];
    return configuration;
  }

  
  bool Powertrain::saveConfiguration(uint8_t * data, uint16_t size)
  {
    if(size >= 13u)
    {
      auto esc_configuration = esc.getConfiguration();
      data[0] = esc_configuration.minimum_pulse_width & 0x00ff;
      data[1] = (esc_configuration.minimum_pulse_width & 0xff00) >> 8;
      data[2] = esc_configuration.center_pulse_width & 0x00ff;
      data[3] = (esc_configuration.center_pulse_width & 0xff00) >> 8;
      data[4] = esc_configuration.maximum_pulse_width & 0x00ff;
      data[5] = (esc_configuration.maximum_pulse_width & 0xff00) >> 8;
      data[6] = gearbox_pulse_widths[0] & 0x00ff;
      data[7] = (gearbox_pulse_widths[0] & 0xff00) >> 8;
      data[8] = gearbox_pulse_widths[1] & 0x00ff;
      data[9] = (gearbox_pulse_widths[1] & 0xff00) >> 8;
      data[10] = gearbox_pulse_widths[2] & 0x00ff;
      data[11] = (gearbox_pulse_widths[2] & 0xff00) >> 8;
      data[12] = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7] ^ data[8] ^ data[9]
                 ^ data[10] ^ data[11];
      return true;
    }
    else
    {
      return false;
    }

  }


  bool Powertrain::loadConfiguration(uint8_t * data, uint16_t size)
  {
    if(size >= 13u && data[12] == (data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5]
                                   ^ data[6] ^ data[7] ^ data[8] ^ data[9] ^ data[10] ^ data[11]))
    {
      auto pulse_width_reverse = uint16_t(data[0] | (data[1] << 8));
      auto pulse_width_stop = uint16_t(data[2] | (data[3] << 8));
      auto pulse_width_forward = uint16_t(data[4] | (data[5] << 8));
      decltype(esc)::Configuration configuration
        {pulse_width_reverse, pulse_width_stop, pulse_width_forward, pulse_width_stop, true};
      esc.setConfiguration(configuration);
      esc.setPosition(current_throttle_setting);
      gearbox_pulse_widths[0] = uint16_t(data[6] | (data[7] << 8));
      gearbox_pulse_widths[1] = uint16_t(data[8] | (data[9] << 8));
      gearbox_pulse_widths[2] = uint16_t(data[10] | (data[11] << 8));
      return true;
    }
    else
    {
      return false;
    }

  }

}