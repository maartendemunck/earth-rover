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
    if(state != lighting.turn_signal_right)
    {
      lighting.turn_signal_right = state;
      lighting_changed = true;
    }
  }


  void CarState::setTurnSignalLeft(bool state)
  {
    if(state != lighting.turn_signal_left)
    {
      lighting.turn_signal_left = state;
      lighting_changed = true;
    }
  }


  void CarState::setDippedBeam(bool state)
  {
    if(state != lighting.dipped_beam)
    {
      lighting.dipped_beam = state;
      lighting_changed = true;
    }
  }


  void CarState::setHighBeam(bool state)
  {
    if(state != lighting.high_beam)
    {
      lighting.high_beam = state;
      lighting_changed = true;
    }
  }


  void CarState::setHazardFlashers(bool state)
  {
    if(state != lighting.hazard_flashers)
    {
      lighting.hazard_flashers = state;
      lighting_changed = true;
    }
  }

}