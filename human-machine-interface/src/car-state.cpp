//! Car state (digital twin) for the Earth Rover (implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include <Arduino.h>
#include "car-state.hpp"


namespace earth_rover_hmi
{

  CarState::CarState()
  :
    speedometer {1000u},
    orientation {1000u},
    location {5000u},
    altitude {5000u}
  {
    ;
  }


  void CarState::setSteeringInput(int16_t steering)
  {
    drive.steering = limit_value(steering, int16_t(-1000), int16_t(1000));
    if(lighting.turn_signal_right && !lighting.turn_signal_left)
    {
      // If the steering input is moved more than 50% right, unlock the right turn signal.
      if(steering > 500 && turn_signal_free != 1)
      {
        turn_signal_free = 1;
      }
      // If the steering input is moved back to the center (less than 10% right), shut off the right turn signal.
      else if(turn_signal_free == 1 && steering < 100)
      {
        setTurnSignalRight(false);
        turn_signal_free = 0;
        turn_signal_right_cancelled = true;
      }
    }
    else if(lighting.turn_signal_left && !lighting.turn_signal_right)
    {
      // If the steering input is moved more than 50% left, unlock the left turn signal.
      if(steering < -500 && turn_signal_free != -1)
      {
        turn_signal_free = -1;
      }
      // If the steering input is moved back to the center (less than 10% left), shut off the left turn signal.
      else if(turn_signal_free == -1 && steering > -100)
      {
        setTurnSignalLeft(false);
        turn_signal_free = 0;
        turn_signal_left_cancelled = true;
      }
    }
    else if(!lighting.turn_signal_right && !lighting.turn_signal_left)
    {
      // If no turn signal is active, deactivate the auto shut off function.
      if(turn_signal_free != 0)
      {
        turn_signal_free = 0;
      }
    }
  }


  void CarState::setThrottleInput(int16_t throttle)
  {
    drive.throttle = limit_value(throttle, int16_t(-1000), int16_t(1000));
  }


  void CarState::setGearboxInput(int16_t gearbox)
  {
    // If the input stick is moved back to the center zone, unlock the gearbox.
    if(gearbox_locked && gearbox_locked * gearbox < 250)
    {
      gearbox_locked = 0;
    }
    // If the gearbox is unlocked and the input stick is moved to the end of the range, shift.
    if(gearbox < -750 && !gearbox_locked && drive.gear > 0)
    {
      drive.gear --;
      gearbox_locked = -1;
    }
    else if(gearbox > 750 && !gearbox_locked && drive.gear < 2)
    {
      drive.gear ++;
      gearbox_locked = 1;
    }
  }


  void CarState::setTurnSignalRight(bool state)
  {
      lighting.turn_signal_right = state;
  }


  bool CarState::getTurnSignalRightCancelled(bool reset)
  {
    auto return_value = turn_signal_right_cancelled;
    if(reset)
    {
      turn_signal_right_cancelled = false;
    }
    return return_value;
  }


  void CarState::setTurnSignalLeft(bool state)
  {
      lighting.turn_signal_left = state;
  }


  bool CarState::getTurnSignalLeftCancelled(bool reset)
  {
    auto return_value = turn_signal_left_cancelled;
    if(reset)
    {
      turn_signal_left_cancelled = false;
    }
    return return_value;
  }


  void CarState::setDippedBeam(bool state)
  {
      lighting.dipped_beam = state;
  }


  void CarState::setHighBeam(bool state)
  {
      lighting.high_beam = state;
  }


  void CarState::setHazardFlashers(bool state)
  {
      lighting.hazard_flashers = state;
  }

}