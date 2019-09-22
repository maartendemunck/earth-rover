#ifndef __VCU_COMMUNICATOR__
#define __VCU_COMMUNICATOR__


#include <functional>
#include <Arduino.h>
#include <RF24.h>
#undef printf  // RF24.h defined 'printf' as 'Serial.printf', expanding 'Serial.printf' to 'Serial.Serial.printf'.
#include "vcu.hpp"


namespace earth_rover
{

  class VcuCommunicator
  {
    private:

      enum class RequestMessageType: uint8_t { Control = 0x00, RequestState = 0x10 };
      enum class ResponseMessageType: uint8_t { Orientation = 0x91, Location = 0x92, Altitude = 0x93 };

      RF24 nrf24l01_device;
      static constexpr uint8_t nrf24l01_payload_size {9u};
      // This sequence seems to work reliably with the Velleman nRF24L01+ modules.
      static constexpr uint8_t nrf24l01_fhss_channels[]
        {  0,  4,  8, 12, 16, 20, 24, 28, 32, 36,  1,  5,  9, 13, 17, 21, 25, 29, 33, 37,
           2,  6, 10, 14, 18, 22, 26, 30, 34, 38,  3,  7, 11, 15, 19, 23, 27, 31, 35, 39 };
      uint8_t nrf24l01_fhss_channel_index;
      static constexpr uint8_t update_interval {50u};
      elapsedMillis since_channel_change;
      bool channel_changed;
      uint32_t fhss_timeout {(sizeof(nrf24l01_fhss_channels) / sizeof(nrf24l01_fhss_channels[0])) * update_interval};
      bool fhss_synced;
      elapsedMillis since_last_message;

      Vcu & vcu;

    public:

      VcuCommunicator(uint8_t ce_pin, uint8_t csn_pin, Vcu & vcu);

      void setup();
      void spinOnce();

    private:

      bool sendOrientationMessage();
      bool sendLocationMessage();
      bool sendAltitudeMessage();
      bool sendMessage(uint8_t buffer[nrf24l01_payload_size]);
      void changeChannel();
  };

}

#endif