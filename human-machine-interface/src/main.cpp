#include <Arduino.h>
#include "car-configuration.hpp"
#include "car-state.hpp"
#include "hmi-communicator.hpp"
#include "hmi-display.hpp"
#include "limit-value.hpp"


// #define SERIAL_DEBUG


// Pin assignments.
constexpr uint8_t input_pins[] = {2, 3, 4, 5};
constexpr uint8_t rf24_ce_pin = 6;
constexpr uint8_t spi_sck_pin = 14;
constexpr uint8_t rf24_csn_pin = 15;


// Global objects.
earth_rover::CarConfiguration car_configuration {earth_rover::ServoConfiguration{1000u, 1500u, 2000u, true, 1u},
                                                 earth_rover::ServoConfiguration{1000u, 1500u, 2000u, true, 4u},
                                                 earth_rover::ServoConfiguration{1150u, 1500u, 1850u, true, 3u},
                                                 earth_rover::RadioConfiguration{1u, 1u}};
earth_rover::CarState car_state;
earth_rover::HmiCommunicator communicator {rf24_ce_pin, rf24_csn_pin, car_configuration, car_state};
earth_rover::HmiDisplay<decltype(Serial1)> display {Serial1, car_configuration, car_state};


void setup()
{
  // Setup debug console.
  #ifdef SERIAL_DEBUG
    Serial.begin(9600);
    while(!Serial)
    {
      ;
    }
  #endif
  // Initialisation started, switch on built in LED.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  // Configure SPI.
  SPI.setSCK(spi_sck_pin);  // Use the default SCK pin 13 as status LED.
  // Configure nRF24L01+ communication module.
  communicator.setup();
  // Setup Nextion display.
  display.setup();
  display.publishVelocity(10.);
  // Initialisation finished, switch off built in LED.
  digitalWrite(LED_BUILTIN, 0);
}


void loop()
{
  //digitalWrite(LED_BUILTIN, 1);
  // Read analog inputs.
  int16_t inputs[4];
  for(uint8_t channel = 0; channel < 4; ++ channel)
  {
    inputs[channel] = earth_rover::limit_value((analogRead(input_pins[channel]) - 512) * 2, -1000, 1000);
    if(channel % 2 == 0) {
      inputs[channel] = -inputs[channel];
    }
  }

  // Set drive commands.
  // TODO: get channels from configuration;
  constexpr uint8_t steering_channel = 1;
  constexpr uint8_t throttle_channel = 4;
  constexpr uint8_t gearbox_channel = 3;
  car_state.setSteeringInput(inputs[steering_channel - 1]);
  car_state.setThrottleInput(inputs[throttle_channel - 1]);
  car_state.setGearboxInput(inputs[gearbox_channel - 1]);

  // Send drive command.
  communicator.spinOnce();

  // Update display.
  display.spinOnce();
}


void serialEvent1()
{
  display.receiveData();
}