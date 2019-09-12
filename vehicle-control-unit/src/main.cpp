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


earth_rover::VcuCommunicator communicator {rf24_ce_pin, rf24_csn_pin};

earth_rover::ConfiguredServo steering_servo {steering_servo_pin, 2000u, 1000u, 1500u, true};
earth_rover::ConfiguredServo throttle_servo {throttle_servo_pin};
earth_rover::ConfiguredServo gearbox_servo {gearbox_servo_pin, 1150u, 1850u, 1460u, true};
earth_rover::Lighting automotive_lighting { head_lamp_pin, tail_lamp_pin, turn_signal_right_pin, turn_signal_left_pin };


void TimeoutCallback()
{
  throttle_servo.setPosition(0);
  automotive_lighting.setHazardFlashers(true);
  automotive_lighting.setStopLamps(true);
}


void ControlMessageCallback(int16_t steering, int16_t throttle, int8_t gearbox, int8_t lighting)
{
  // TODO: Move this function and these variables to the CarState object.
  static bool is_driving = false;
  static elapsedMillis since_driving;

  steering_servo.setPosition(steering);
  throttle_servo.setPosition(throttle);
  if(!is_driving && abs(throttle) >= 150)
  {
    since_driving = 0;
    is_driving = true;
  }
  else if(abs(throttle) < 100)
  {
    is_driving = false;
  }
  
  switch(gearbox)
  {
    case 1:  // Low
      if(is_driving && since_driving > 500u)
      {
        gearbox_servo.setPosition(-1000);
      }
      break;
    case 2:  // High
      if(is_driving && since_driving > 200u)
      {
        gearbox_servo.setPosition(1000);
      }
      break;
    default:  // Neutral
      gearbox_servo.setPosition(0);
      break;
  }
  automotive_lighting.setTurnSignalRight(lighting & 0x01);
  automotive_lighting.setTurnSignalLeft(lighting & 0x02);
  automotive_lighting.setDippedBeam(lighting & 0x04);
  automotive_lighting.setHighBeam(lighting & 0x08);
  automotive_lighting.setHazardFlashers(lighting & 0x10);
  automotive_lighting.setStopLamps(throttle > -50 && throttle < 50);
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
  // Configure servos (the steering servo requires an explicit setPosition (on top of the one in the setup function)).
  steering_servo.setup();
  steering_servo.setPosition(0);
  throttle_servo.setup();
  throttle_servo.setPosition(0);
  gearbox_servo.setup();
  gearbox_servo.setPosition(0);
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
  automotive_lighting.spinOnce();
}