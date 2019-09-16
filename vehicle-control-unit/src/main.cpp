#include <Arduino.h>
#include "configured-servo.hpp"
#include "lighting.hpp"
#include "vcu-communicator.hpp"


// #define SERIAL_DEBUG


// Pin assignments.
constexpr uint8_t head_lamp_pin = 3u;
constexpr uint8_t tail_lamp_pin = 4u;
constexpr uint8_t turn_signal_right_pin = 5u;
constexpr uint8_t turn_signal_left_pin = 6u;
constexpr uint8_t rf24_ce_pin = 10u;
constexpr uint8_t spi_sck_pin = 14u;
constexpr uint8_t rf24_csn_pin = 15u;
constexpr uint8_t steering_servo_pin = 20u;
constexpr uint8_t throttle_servo_pin = 21u;
constexpr uint8_t gearbox_servo_pin = 22u;


earth_rover::Vcu vcu {steering_servo_pin, throttle_servo_pin, gearbox_servo_pin, head_lamp_pin, tail_lamp_pin,
    turn_signal_right_pin, turn_signal_left_pin};
earth_rover::VcuCommunicator communicator {rf24_ce_pin, rf24_csn_pin, vcu};



void setup()
{
  // Initialisation started, switch on built in LED.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  // Configure SPI.
  SPI.setSCK(spi_sck_pin);  // Use the default SCK pin 13 as status LED.
  // Setup the VCU.
  vcu.setup();
  // Configure nRF24L01+ communication module.
  communicator.setup();
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
  vcu.spinOnce();
}