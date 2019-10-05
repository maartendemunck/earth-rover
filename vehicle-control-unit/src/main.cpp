#include <Arduino.h>
#include <i2c_t3.h>
#include "lighting.hpp"
#include "neogps-wrapper.hpp"
#include "powertrain.hpp"
#include "steering-servo.hpp"
#include "vcu.hpp"
#include "vcu-communicator.hpp"


// #define SERIAL_DEBUG


// Pin assignments.
constexpr uint8_t steering_servo_pin = 20u;
constexpr uint8_t esc_pin = 21u;
constexpr uint8_t gearbox_servo_pin = 22u;

constexpr uint8_t head_lamp_pin = 3u;
constexpr uint8_t tail_lamp_pin = 4u;
constexpr uint8_t turn_signal_right_pin = 5u;
constexpr uint8_t turn_signal_left_pin = 6u;

constexpr i2c_t3 & imu_i2c = Wire;
constexpr uint8_t imu_i2c_scl_pin = 19u;
constexpr uint8_t imu_i2c_sda_pin = 18u;

constexpr uint8_t rf24_ce_pin = 10u;
constexpr uint8_t spi_sck_pin = 14u;
constexpr uint8_t rf24_csn_pin = 15u;

// Components.
earth_rover_vcu::SteeringServo steering_servo {steering_servo_pin};
earth_rover_vcu::Powertrain powertrain {esc_pin, gearbox_servo_pin};
earth_rover_vcu::NeoGpsWrapper<HardwareSerial> gps {Serial1, 0, 1};
earth_rover_vcu::Vcu<decltype(steering_servo), decltype(powertrain), decltype(gps)> vcu
  { head_lamp_pin, tail_lamp_pin, turn_signal_right_pin, turn_signal_left_pin, imu_i2c, imu_i2c_scl_pin, imu_i2c_sda_pin,
    steering_servo, powertrain, gps};
earth_rover_vcu::VcuCommunicator<decltype(vcu)> communicator {rf24_ce_pin, rf24_csn_pin, vcu};


void setup()
{
  // Initialisation started, switch on built in LED.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  // Setup debug console.
  #ifdef SERIAL_DEBUG
    Serial.begin(9600);
    while(!Serial)
    {
      ;
    }
  #endif
  // Configure SPI.
  SPI.setSCK(spi_sck_pin);  // Use the default SCK pin 13 as status LED.
  // Setup the VCU.
  vcu.setup();
  // Configure nRF24L01+ communication module.
  communicator.setup();
  // Initialisation finished, switch off built in LED.
  digitalWrite(LED_BUILTIN, 0);
}


void loop()
{
  communicator.spinOnce();
  vcu.spinOnce();
}