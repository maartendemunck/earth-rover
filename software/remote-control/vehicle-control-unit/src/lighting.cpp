// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#include "lighting.hpp"

namespace earth_rover {

    Lighting::Lighting(uint8_t head_lamp_pin_number, uint8_t tail_lamp_pin_number,
                       uint8_t stop_lamp_pin_number, uint8_t turn_signal_right_pin_number,
                       uint8_t turn_signal_left_pin_number)
        : head_lamp_pin_number{head_lamp_pin_number}, tail_lamp_pin_number{tail_lamp_pin_number},
          stop_lamp_pin_number{stop_lamp_pin_number},
          turn_signal_right_pin_number{turn_signal_right_pin_number},
          turn_signal_left_pin_number{turn_signal_left_pin_number} {
        ;
    }

    Lighting::~Lighting() {
        pinMode(head_lamp_pin_number, INPUT);
        pinMode(tail_lamp_pin_number, INPUT);
        pinMode(stop_lamp_pin_number, INPUT);
        pinMode(turn_signal_right_pin_number, INPUT);
        pinMode(turn_signal_left_pin_number, INPUT);
    }

    void Lighting::setup() {
        pinMode(head_lamp_pin_number, OUTPUT);
        pinMode(tail_lamp_pin_number, OUTPUT);
        pinMode(stop_lamp_pin_number, OUTPUT);
        pinMode(turn_signal_right_pin_number, OUTPUT);
        pinMode(turn_signal_left_pin_number, OUTPUT);
    }

    void Lighting::spinOnce() {
        if(time_since_turn_signal_flipped >= turn_signal_period) {
            turn_signal_flipped_state = !turn_signal_flipped_state;
            if(hazard_flashers_state == true) {
                digitalWrite(turn_signal_right_pin_number, turn_signal_flipped_state);
                digitalWrite(turn_signal_left_pin_number, turn_signal_flipped_state);
            }
            else if(turn_signal_right_state == true && turn_signal_left_state == false) {
                digitalWrite(turn_signal_right_pin_number, turn_signal_flipped_state);
            }
            else if(turn_signal_left_state == true && turn_signal_right_state == false) {
                digitalWrite(turn_signal_left_pin_number, turn_signal_flipped_state);
            }
            time_since_turn_signal_flipped -= turn_signal_period;
        }
    }

    void Lighting::setTurnSignalRight(bool state) {
        if(state != turn_signal_right_state) {
            turn_signal_right_state = state;
            updateSteadyLighting();
            turn_signal_flipped_state = false;
            time_since_turn_signal_flipped = turn_signal_period;
        }
    }

    void Lighting::setTurnSignalLeft(bool state) {
        if(state != turn_signal_left_state) {
            turn_signal_left_state = state;
            updateSteadyLighting();
            turn_signal_flipped_state = false;
            time_since_turn_signal_flipped = turn_signal_period;
        }
    }

    void Lighting::setDippedBeam(bool state) {
        dipped_beam_state = state;
        updateSteadyLighting();
    }

    void Lighting::setHighBeam(bool state) {
        high_beam_state = state;
        updateSteadyLighting();
    }

    void Lighting::setStopLamps(bool state) {
        stop_lamps_state = state;
        updateSteadyLighting();
    }

    void Lighting::setHazardFlashers(bool state) {
        if(state != hazard_flashers_state) {
            hazard_flashers_state = state;
            updateSteadyLighting();
            turn_signal_flipped_state = false;
            time_since_turn_signal_flipped = turn_signal_period;
        }
    }

    void Lighting::updateSteadyLighting() {
        if(high_beam_state == true) {
            analogWrite(head_lamp_pin_number, 256);
        }
        else if(dipped_beam_state == true) {
            analogWrite(head_lamp_pin_number, 100);
        }
        else {
            analogWrite(head_lamp_pin_number, 0);
        }
        digitalWrite(tail_lamp_pin_number, dipped_beam_state);
        digitalWrite(stop_lamp_pin_number, stop_lamps_state);
        if(turn_signal_right_state == false && hazard_flashers_state == false) {
            digitalWrite(turn_signal_right_pin_number, 0);
        }
        if(turn_signal_left_state == false && hazard_flashers_state == false) {
            digitalWrite(turn_signal_left_pin_number, 0);
        }
    }

}  // namespace earth_rover