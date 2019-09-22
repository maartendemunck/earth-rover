#include <Arduino.h>
#include "car-state.hpp"


namespace earth_rover
{

  CarState::CarState()
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


  void CarState::setOrientation(const Orientation & new_orientation)
  {
    orientation = new_orientation;
    orientation_changed = true;
  }


  const CarState::Orientation & CarState::getOrientation(bool reset)
  {
    if(reset)
    {
      orientation_changed = false;
    }
    return orientation;
  }


  void CarState::setLocation(const Location & new_location)
  {
    location = new_location;
    location_changed = true;
  }


  const CarState::Location & CarState::getLocation(bool reset)
  {
    if(reset)
    {
      location_changed = false;
    }
    return location;
  }
  
  
  void CarState::setAltitude(int32_t new_altitude)
  {
    altitude = new_altitude;
    altitude_changed = true;
  }
  
  
  int32_t CarState::getAltitude(bool reset)
  {
    if(reset)
    {
      altitude_changed = false;
    }
    return altitude;
  }

}