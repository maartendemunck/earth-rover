//! Automotive lighting device driver for the Earth Rover's VCU (implementation).
/*!
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */

#include "lighting.hpp"

namespace earth_rover_vcu {

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
        if(since_turn_signal_flipped >= turn_signal_period) {
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
            since_turn_signal_flipped -= turn_signal_period;
        }
    }

    void Lighting::setTurnSignalRight(bool state) {
        if(state != turn_signal_right_state) {
            turn_signal_right_state = state;
            updateSteadyLighting();
            turn_signal_flipped_state = false;
            since_turn_signal_flipped = turn_signal_period;
        }
    }

    void Lighting::setTurnSignalLeft(bool state) {
        if(state != turn_signal_left_state) {
            turn_signal_left_state = state;
            updateSteadyLighting();
            turn_signal_flipped_state = false;
            since_turn_signal_flipped = turn_signal_period;
        }
    }

    void Lighting::setHazardFlashers(bool state) {
        if(state != hazard_flashers_state) {
            hazard_flashers_state = state;
            updateSteadyLighting();
            turn_signal_flipped_state = false;
            since_turn_signal_flipped = turn_signal_period;
        }
    }

    void Lighting::updateSteadyLighting() {
        // Head lights at full intensity when high beam is on or at half intensity when dipped beam
        // is on.
        if(high_beam_state == true) {
            analogWrite(head_lamp_pin_number, 256);
        }
        else if(dipped_beam_state == true) {
            analogWrite(head_lamp_pin_number, 100);
        }
        else {
            analogWrite(head_lamp_pin_number, 0);
        }
        // Set tail lamps and stop lamps. Combining both in a single LED will be handled by the
        // hardware.
        digitalWrite(tail_lamp_pin_number, dipped_beam_state);
        digitalWrite(stop_lamp_pin_number, stop_lamps_state);
        // Shut turn signals off if requested (on will be handled by the spinOnce loop).
        if(turn_signal_right_state == false && hazard_flashers_state == false) {
            digitalWrite(turn_signal_right_pin_number, 0);
        }
        if(turn_signal_left_state == false && hazard_flashers_state == false) {
            digitalWrite(turn_signal_left_pin_number, 0);
        }
    }

}  // namespace earth_rover_vcu