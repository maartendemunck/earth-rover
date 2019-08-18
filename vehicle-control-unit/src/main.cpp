#include <Arduino.h>
#include "configured-servo.hpp"
#include "vcu-communicator.hpp"


// #define SERIAL_DEBUG


// Pin assignments.
constexpr uint8_t rf24_ce_pin = 10u;
constexpr uint8_t spi_sck_pin = 14u;
constexpr uint8_t rf24_csn_pin = 15u;
constexpr uint8_t steering_servo_pin = 20u;
constexpr uint8_t throttle_servo_pin = 21u;
constexpr uint8_t gearbox_servo_pin = 22u;


earth_rover::VcuCommunicator communicator {rf24_ce_pin, rf24_csn_pin};

earth_rover::ConfiguredServo steering_servo {steering_servo_pin, 2000u, 1000u, 1500u, true};
earth_rover::ConfiguredServo throttle_servo {throttle_servo_pin};
earth_rover::ConfiguredServo gearbox_servo {gearbox_servo_pin, 1150u, 1850u, 1460u, true};


void TimeoutCallback()
{
  throttle_servo.setPosition(0);
}


void ControlMessageCallback(int16_t steering, int16_t throttle, int8_t gearbox, int8_t lighting)
{
  steering_servo.setPosition(steering);
  throttle_servo.setPosition(throttle);
  switch(gearbox)
  {
    case 1:  // Low
      gearbox_servo.setPosition(-1000);
      break;
    case 2:  // High
      gearbox_servo.setPosition(1000);
      break;
    default:  // Neutral
      gearbox_servo.setPosition(0);
      break;
  }
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
  communicator.setTimeoutCallback(&TimeoutCallback, 500u);
  communicator.setControlMessageCallback(&ControlMessageCallback);
  // Configure servos.
  steering_servo.setup();
  throttle_servo.setup();
  gearbox_servo.setup();
  // Setup debug console.
  #ifdef SERIAL_DEBUG
    Serial.begin(9600);
    while(!Serial)
    {
      ;
    }
  #endif
  // Initialisation finished, switch off built in LED.
  digitalWrite(LED_BUILTIN, 0);
}


void loop()
{
  communicator.spinOnce();
}