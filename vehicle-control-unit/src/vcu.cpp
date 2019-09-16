#include "vcu.hpp"


namespace earth_rover
{

  Vcu::Vcu (uint8_t steering_servo_pin, uint8_t throttle_servo_pin, uint8_t gearbox_servo_pin,
      uint8_t head_lamp_pin, uint8_t tail_lamp_pin, uint8_t turn_signal_right_pin, uint8_t turn_signal_left_pin):
    steering_servo {steering_servo_pin, 2000u, 1000u, 1500u, true},
    throttle_servo {throttle_servo_pin, 1000u, 2000u, 1500u, true},
    gearbox_servo {gearbox_servo_pin, 1150u, 1850u, 1480u, true},
    automotive_lighting {head_lamp_pin, tail_lamp_pin, turn_signal_right_pin, turn_signal_left_pin}
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