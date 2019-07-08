#include <Arduino.h>
#include <hmi-communicator.hpp>


// Pin assignments.
constexpr uint8_t input_steering_pin = 2;
constexpr uint8_t input_throttle_pin = 3;
constexpr uint8_t rf24_ce_pin = 6;
constexpr uint8_t spi_sck_pin = 14;
constexpr uint8_t rf24_csn_pin = 15;


earth_rover::HmiCommunicator<rf24_ce_pin, rf24_csn_pin, 8u> communicator;


template<typename T>
T limit_value(T value, T lower, T upper)
{
  if(value < lower)
  {
    return lower;
  }
  else if(value > upper)
  {
    return upper;
  }
  else {
    return value;
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
  // Setup debug console.
  Serial.begin(9600);
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
  digitalWrite(LED_BUILTIN, 1);
  int16_t steering = limit_value((analogRead(input_steering_pin) - 512) * 2, -1000, 1000);
  int16_t throttle = limit_value((analogRead(input_throttle_pin) - 512) * 2, -1000, 1000);
  Serial.printf("sending: steering: %4u , throttle = %4u ... ", steering, throttle);
  communicator.sendControlMessage(steering, throttle);
  delay(100);
}