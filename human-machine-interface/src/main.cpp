#include <Arduino.h>
#include "car-configuration.hpp"
#include "hmi-communicator.hpp"
#include "hmi-display.hpp"
#include "limit-value.hpp"


// Pin assignments.
constexpr uint8_t input_steering_pin = 2;
constexpr uint8_t input_throttle_pin = 3;
constexpr uint8_t rf24_ce_pin = 6;
constexpr uint8_t spi_sck_pin = 14;
constexpr uint8_t rf24_csn_pin = 15;


// Global objects.
earth_rover::CarConfiguration car_configuration {earth_rover::ServoConfiguration{1000u, 1500u, 2000u, true, 1u},
                                                 earth_rover::ServoConfiguration{1000u, 1500u, 2000u, true, 4u},
                                                 earth_rover::ServoConfiguration{1150u, 1500u, 1850u, true, 3u},
                                                 earth_rover::RadioConfiguration{202u, 1u, 1u, 101u}};
earth_rover::HmiCommunicator communicator {rf24_ce_pin, rf24_csn_pin};
earth_rover::HmiDisplay<decltype(Serial1)> display {Serial1, car_configuration};


void setup()
{
  // BEGIN DEBUG.
  // Setup debug console.
  Serial.begin(9600);
  while(!Serial)
  {
    ;
  }
  // END DEBUG.
  // Initialisation started, switch on built in LED.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  // Configure SPI.
  SPI.setSCK(spi_sck_pin);  // Use the default SCK pin 13 as status LED.
  // Configure nRF24L01+ communication module.
  communicator.setup();
  // Setup Nextion display.
  display.setup();
  // Initialisation finished, switch off built in LED.
  digitalWrite(LED_BUILTIN, 0);
}


void loop()
{
  digitalWrite(LED_BUILTIN, 1);
  int16_t steering = earth_rover::limit_value((analogRead(input_steering_pin) - 512) * 2, -1000, 1000);
  int16_t throttle = earth_rover::limit_value((analogRead(input_throttle_pin) - 512) * 2, -1000, 1000);
  //Serial.printf("sending: steering: %4i , throttle = %4i ...\n", steering, throttle);
  communicator.sendControlMessage(steering, throttle);
  digitalWrite(LED_BUILTIN, 0);
  delay(50);
}


void serialEvent1()
{
  display.receiveData();
}