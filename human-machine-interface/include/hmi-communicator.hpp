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
      static constexpr uint8_t nrf24l01_payload_size {9u};
      // This sequence seems to work reliably with the Velleman nRF24L01+ modules.
      static constexpr uint8_t nrf24l01_fhss_channels[]
        {  0,  4,  8, 12, 16, 20, 24, 28, 32, 36,  1,  5,  9, 13, 17, 21, 25, 29, 33, 37,
           2,  6, 10, 14, 18, 22, 26, 30, 34, 38,  3,  7, 11, 15, 19, 23, 27, 31, 35, 39 };
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