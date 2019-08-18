#ifndef __HMI_COMMUNICATOR__
#define __HMI_COMMUNICATOR__


#include <cstdint>
#include <RF24.h>
#undef printf  // RF24.h defined 'printf' as 'Serial.printf', expanding 'Serial.printf' to 'Serial.Serial.printf'.
#include "car-configuration.hpp"
#include "car-state.hpp"


namespace earth_rover
{

  class HmiCommunicator
  {
    private:

      enum class MessageType: uint8_t { Control = 0x00 };

      RF24 nrf24l01;
      static constexpr uint8_t nrf24l01_payload_size {10u};

      CarConfiguration & car_configuration;
      CarState & car_state;

    public:

      HmiCommunicator(uint8_t ce_pin, uint8_t csn_pin, CarConfiguration & car_configuration, CarState & car_state);
      void setup();
      void spinOnce();
      bool sendControlMessage();

    private:

      bool sendMessage(uint8_t buffer[]);

  };

}

#endif