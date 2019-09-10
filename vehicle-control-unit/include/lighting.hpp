#ifndef __LIGHTING__
#define __LIGHTING__


#include <cstdint>
#include <Arduino.h>


namespace earth_rover
{

  class Lighting
  {

    private:

      uint8_t head_lamp_pin_number;
      uint8_t tail_lamp_pin_number;
      uint8_t turn_signal_right_pin_number;
      uint8_t turn_signal_left_pin_number;

      bool turn_signal_right_state;
      bool turn_signal_left_state;
      bool dipped_beam_state;
      bool high_beam_state;
      bool hazard_flashers_state;
      bool stop_lamps_state;

      static constexpr uint16_t turn_signal_period = 750u;
      elapsedMillis since_turn_signal_flipped;
      bool turn_signal_flipped_state;

    public:

      Lighting(uint8_t head_lamp_pin_number, uint8_t tail_lamp_pin_number,
               uint8_t turn_signal_right_pin_number, uint8_t turn_signal_lefLigtt_pin_number);
      ~Lighting();
      void setup();
      void spinOnce();
      void setTurnSignalRight(bool state);
      void setTurnSignalLeft(bool state);
      void setDippedBeam(bool state) { dipped_beam_state = state; updateSteadyLighting(); };
      void setHighBeam(bool state) { high_beam_state = state; updateSteadyLighting(); };
      void setHazardFlashers(bool state);
      void setStopLamps(bool state) { stop_lamps_state = state; updateSteadyLighting(); };

    private:

      void updateSteadyLighting();
  };

}

#endif