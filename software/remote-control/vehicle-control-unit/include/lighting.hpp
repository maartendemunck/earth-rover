// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__LIGHTING__
#define __EARTH_ROVER__LIGHTING__

#include <Arduino.h>
#include <cstdint>

namespace earth_rover {

    class Lighting {

      private:
        uint8_t head_lamp_pin_number;
        uint8_t tail_lamp_pin_number;
        uint8_t stop_lamp_pin_number;
        uint8_t turn_signal_right_pin_number;
        uint8_t turn_signal_left_pin_number;

        bool turn_signal_right_state{false};
        bool turn_signal_left_state{false};
        bool dipped_beam_state{false};
        bool high_beam_state{false};
        bool hazard_flashers_state{false};
        bool stop_lamps_state{false};

        static constexpr uint16_t turn_signal_period = 750u;
        elapsedMillis time_since_turn_signal_flipped;
        bool turn_signal_flipped_state;

      public:
        Lighting(uint8_t head_lamp_pin_number, uint8_t tail_lamp_pin_number,
                 uint8_t stop_lamp_pin_number, uint8_t turn_signal_right_pin_number,
                 uint8_t turn_signal_left_pin_number);
        ~Lighting();

        void setup();
        void spinOnce();

        void setTurnSignalRight(bool state);
        void setTurnSignalLeft(bool state);
        void setDippedBeam(bool state);
        void setHighBeam(bool state);
        void setStopLamps(bool state);
        void setHazardFlashers(bool state);

      private:
        void updateSteadyLighting();
    };

}  // namespace earth_rover

#endif