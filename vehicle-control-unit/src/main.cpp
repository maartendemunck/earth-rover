#include <Arduino.h>
#include "configured_servo.hpp"
#include "vcu-communicator.hpp"


// Pin assignments.
constexpr uint8_t rf24_ce_pin = 10u;
constexpr uint8_t spi_sck_pin = 14u;
constexpr uint8_t rf24_csn_pin = 15u;
constexpr uint8_t steering_servo_pin = 20u;
constexpr uint8_t throttle_servo_pin = 21u;


earth_rover::VcuCommunicator communicator {rf24_ce_pin, rf24_csn_pin};

earth_rover::ConfiguredServo steering_servo {steering_servo_pin};
earth_rover::ConfiguredServo throttle_servo {throttle_servo_pin};


void ControlMessageCallback(int16_t steering, int16_t throttle)
{
  steering_servo.setPosition(steering);
  throttle_servo.setPosition(throttle);
}


void setup()
{
  // Initialisation started, switch on built in LED.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  // Configure SPI.
  SPI.setSCK(spi_sck_pin);  // Use the default SCK pin 13 as status LED.
  // Configure nRF24L01+ communication module.
  communicator.setup();
  communicator.setControlMessageCallback(&ControlMessageCallback);
  // Configure servos.
  steering_servo.setup();
  throttle_servo.setup();
  // Setup debug console.
  Serial.begin(9600);
  // DEBUG.
  /*
  while(!Serial)
  {
    ;
  }
  */
  // END DEBUG.
  // Initialisation finished, switch off built in LED.
  digitalWrite(LED_BUILTIN, 0);
}


void loop()
{
  communicator.spinOnce();
}