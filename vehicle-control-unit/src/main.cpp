//! Earth Rover VCU (vehicle control unit) sketch for a Teensy 3.2.
/*!
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include <Arduino.h>
#include <i2c_t3.h>

#include "steering-servo.hpp"
#include "powertrain.hpp"
#include "lighting.hpp"
#include "position-encoder.hpp"
#include "adafruit-bno055-wrapper.hpp"
#include "neogps-wrapper.hpp"
#include "vcu.hpp"
#include "vcu-communicator.hpp"
#include "vcu-configuration-manager.hpp"


// #define SERIAL_DEBUG
// #define RESET_EEPROM_MEMORY


/*
 *  Pin assignments.
 */

constexpr uint8_t steering_servo_pin = 20u;    //!< I/O pin used to control the steering servo.
constexpr uint8_t esc_pin = 21u;               //!< I/O pin used to control the electronic speed controller (ESC).
constexpr uint8_t gearbox_servo_pin = 22u;     //!< I/O pin used to control the gearbox servo.
constexpr uint8_t head_lamp_pin = 3u;          //!< I/O pin used to control the head lamps.
constexpr uint8_t tail_lamp_pin = 4u;          //!< I/O pin used to control the tail lamps.
constexpr uint8_t turn_signal_right_pin = 5u;  //!< I/O pin used to control the right turn signal.
constexpr uint8_t turn_signal_left_pin = 6u;   //!< I/O pin used to control the left turn signal.

constexpr i2c_t3 & imu_i2c = Wire;             //!< I²C interface used by the BNO055 IMU.
constexpr uint8_t imu_i2c_scl_pin = 19u;       //!< I/O pin used for the I²C interface's SCL signal.
constexpr uint8_t imu_i2c_sda_pin = 18u;       //!< I/O pin used for the I²C interface's SDA signal.

constexpr uint8_t spi_sck_pin = 14u;           //!< I/O pin used for the SPI's SCK signal.
constexpr uint8_t rf24_ce_pin = 10u;           //!< I/O pin used for the nRF24L01+'s CE signal.
constexpr uint8_t rf24_csn_pin = 15u;          //!< I/O pin used for the nRF24L01+'s CSN signal.


/*
 *  Vehicle systems.
 */

//! Steering servo device driver.
earth_rover_vcu::SteeringServo steering_servo {steering_servo_pin};
//! Powertrain device driver.
earth_rover_vcu::Powertrain powertrain {esc_pin, gearbox_servo_pin};
//! Automotive lighting device driver.
earth_rover_vcu::Lighting lighting {head_lamp_pin, tail_lamp_pin, turn_signal_right_pin, turn_signal_left_pin};
//! Position encoder device driver.
earth_rover_vcu::PositionEncoder<7u, 8u> position_encoder {23000u, 0u, 0u};
//! IMU device driver.
earth_rover_vcu::AdafruitBNO055Wrapper<i2c_t3> imu {imu_i2c, imu_i2c_scl_pin, imu_i2c_sda_pin};
//! GPS device driver.
earth_rover_vcu::NeoGpsWrapper<HardwareSerial> gps {Serial1, 0, 1};

//! VCU subsystem abstraction.
auto vcu = earth_rover_vcu::makeVcu(steering_servo, powertrain, lighting, position_encoder, imu, gps);
//! VCU communicator.
earth_rover_vcu::VcuCommunicator<decltype(vcu)> communicator {rf24_ce_pin, rf24_csn_pin, vcu};
//! VCU configuration.
auto configuration_manager = earth_rover_vcu::makeVcuConfigurationManager(
  steering_servo, powertrain, position_encoder, imu, 0u, 2048u);


/*
 *   Sketch.
 */

//! Initialize the VCU.
void setup()
{
  // Initialisation started, switch on built in LED.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  // Setup debug console (if enabled).
  #ifdef SERIAL_DEBUG
    Serial.begin(9600);
    while(!Serial)
    {
      ;
    }
  #endif
  // Reset EEPROM memory (of enabled).
  #ifdef RESET_EEPROM_MEMORY
    for(uint16_t index = 0; index < 2048; ++ index)
    {
      EEPROM.update(index, 0xffu);
    }
    Serial.println("EEPROM memory cleared.");
    while(true)
    {
      digitalWrite(LED_BUILTIN, 0);
      delay(250);
      digitalWrite(LED_BUILTIN, 1);
      delay(250);
    }
  #endif
  // Configure SPI.
  SPI.setSCK(spi_sck_pin);  // Use the default SCK pin 13 as status LED.
  // Read the configuration stored in EEPROM.
  configuration_manager.setup();
  // Setup the VCU.
  vcu.setup();
  // Configure nRF24L01+ communication module.
  communicator.setup();
  // Initialisation finished, switch off built in LED.
  digitalWrite(LED_BUILTIN, 0);
}

//! Spinning loop.
void loop()
{
  communicator.spinOnce();
  vcu.spinOnce();
  configuration_manager.spinOnce();
}