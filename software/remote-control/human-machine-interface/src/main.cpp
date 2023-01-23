//! Earth Rover HMI (human machine interface) sketch for a Teensy 3.2.
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */

#include "car-state.hpp"
#include "hmi-communicator.hpp"
#include "limit-value.hpp"
#include "nextion-hmi-display.hpp"
#include "servo-state.hpp"
#include <Arduino.h>

// #define SERIAL_DEBUG

/*
 *  Pin assignments.
 */

constexpr uint8_t input_pins[] = {2, 3, 4, 5};  //!< Analog inputs for channels 1, 2, 3 and 4.
constexpr uint8_t spi_sck_pin = 14;             //!< I/O pin used for the SPI's SCK signal.
constexpr uint8_t rf24_ce_pin = 6;              //!< I/O pin used for the nRF24L01+'s CE signal.
constexpr uint8_t rf24_csn_pin = 15;            //!< I/O pin used for the nRF24L01+'s CSN signal.

/*
 *  Subsystems.
 */

//! Steering servo defaults.
earth_rover_hmi::ServoConfigParams steering_servo_defaults{0u, 0u, 0u, 0u, true};
//! ESC defaults.
earth_rover_hmi::ServoConfigParams esc_defaults{3u, 0u, 0u, 0u, true};
//! Gearbox servo pulse widths for the different gears (from low to high).
uint16_t gearbox_servo_pulse_widths[3]{0u, 0u, 0u};
//! Gearbox servo defaults.
earth_rover_hmi::GearboxServoConfigParams<0, 1, 2> gearbox_servo_defaults{
    2u, gearbox_servo_pulse_widths};
//! Radio defaults.
earth_rover_hmi::RadioConfigParams radio_defaults{1u, 1u};
//! Car state (digital twin).
earth_rover_hmi::CarState car_state{std::move(steering_servo_defaults), std::move(esc_defaults),
                                    std::move(gearbox_servo_defaults), std::move(radio_defaults)};
//! HMI communicator.
earth_rover_hmi::HmiCommunicator communicator{rf24_ce_pin, rf24_csn_pin, car_state};
//! HMI display.
/*!
 *  There should be a callback handler calling receiveData for this serial device.
 */
earth_rover_hmi::NextionHmiDisplay<decltype(Serial2)> display{Serial2, car_state};

/*
 *   Sketch.
 */

//! Initialize the HMI.
void setup() {
    // Initialisation started, switch on built in LED.
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);
// Setup debug console.
#ifdef SERIAL_DEBUG
    Serial.begin(9600);
    while(!Serial) {
        ;
    }
#endif
    // Configure the configuration and the digital twin.
    car_state.setup();
    // Configure SPI.
    SPI.setSCK(spi_sck_pin);  // Use the default SCK pin 13 as status LED.
    // Configure nRF24L01+ communication module.
    communicator.setup();
    // Setup Nextion display.
    display.setup();
    // Initialisation finished, switch off built in LED.
    digitalWrite(LED_BUILTIN, 0);
}

//! Spinning loop.
void loop() {
    // Read analog inputs.
    int16_t inputs[4];
    for(uint8_t channel = 0; channel < 4; ++channel) {
        inputs[channel] = earth_rover_hmi::limit_value((analogRead(input_pins[channel]) - 512) * 2,
                                                       -1000, 1000);
        if(channel % 2 == 0) {
            inputs[channel] = -inputs[channel];
        }
    }

    // Set drive commands.
    car_state.setSteeringInput(inputs[car_state.getSteeringConfiguration().input_channel]);
    car_state.setThrottleInput(inputs[car_state.getThrottleConfiguration().input_channel]);
    car_state.setGearboxInput(inputs[car_state.getGearboxConfiguration().input_channel]);

    // Update car configuration and state.
    car_state.spinOnce();

    // Send drive command.
    communicator.spinOnce();

    // Update display.
    display.spinOnce();
}

//! Serial2 callback handler.
/*!
 *  Incoming data on Serial2 is handled by the HMI display.
 */
void serialEvent2() { display.receiveData(); }