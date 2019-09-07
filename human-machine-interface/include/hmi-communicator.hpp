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

      RF24 nrf24l01_device;
      static constexpr uint8_t nrf24l01_payload_size {10u};
      static constexpr uint8_t nrf24l01_fhss_channels[]
        { 76, 56, 28, 74, 16, 50, 62, 22, 90, 49, 10, 30, 32, 72,  2, 37,
          21, 36, 89, 73,  9, 68, 92, 13, 51,  7, 31, 61, 17, 84,  1, 86 };
      uint8_t nrf24l01_fhss_channel_index;
      static constexpr uint8_t update_interval {50u};
      elapsedMillis since_update;
      bool channel_changed;

      CarConfiguration & car_configuration;
      CarState & car_state;

    public:

      HmiCommunicator(uint8_t ce_pin, uint8_t csn_pin, CarConfiguration & car_configuration, CarState & car_state);
      void setup();
      void spinOnce();

    private:

      bool sendControlMessage();
      bool sendMessage(uint8_t buffer[]);
      void changeChannel();

  };

}

#endif