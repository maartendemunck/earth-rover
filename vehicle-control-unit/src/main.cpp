#include <Arduino.h>
#include <vcu-communicator.hpp>


// Pin assignments.
constexpr uint8_t rf24_ce_pin = 10;
constexpr uint8_t spi_sck_pin = 14;
constexpr uint8_t rf24_csn_pin = 15;


earth_rover::VcuCommunicator<rf24_ce_pin, rf24_csn_pin, 8u> communicator;


void ControlMessageCallback(int16_t steering, int16_t throttle)
{
  Serial.printf("received: steering: %4d , throttle = %4d.\n", steering, throttle);
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
  // Setup debug console.
  Serial.begin(9600);
  // DEBUG.
  while(!Serial)
  {
    ;
  }
  // END DEBUG.
  // Initialisation finished, switch off built in LED.
  digitalWrite(LED_BUILTIN, 0);
}


void loop()
{
  communicator.spinOnce();
}