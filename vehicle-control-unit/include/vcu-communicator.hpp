#ifndef __VCU_COMMUNICATOR__
#define __VCU_COMMUNICATOR__


#include <functional>
#include <Arduino.h>
#include <RF24.h>
#undef printf  // RF24.h defined 'printf' as 'Serial.printf', expanding 'Serial.printf' to 'Serial.Serial.printf'.
#include "vcu.hpp"


namespace earth_rover
{

  template <typename Vcu>
  class VcuCommunicator
  {
    private:

      enum class RequestMessageType: uint8_t { Control = 0x00, RequestState = 0x10 };
      enum class ResponseMessageType: uint8_t
      {
        Speedometer = 0x90, Orientation = 0x91, Location = 0x92, Altitude = 0x93
      };

      RF24 nrf24l01_device;
      static constexpr uint8_t nrf24l01_payload_size {9u};
      // This sequence seems to work reliably with the Velleman nRF24L01+ modules.
      const uint8_t nrf24l01_fhss_channels[40]
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

      VcuCommunicator(uint8_t ce_pin, uint8_t csn_pin, Vcu & vcu)
      :
        nrf24l01_device {ce_pin, csn_pin},
        nrf24l01_fhss_channel_index {0u},
        vcu {vcu}
      {
        ;
      }

      void setup()
      {
        // Setup nRF24L01+.
        nrf24l01_device.begin();
        nrf24l01_device.setPALevel(RF24_PA_LOW);
        nrf24l01_device.setDataRate(RF24_250KBPS);
        nrf24l01_device.setChannel(0x01);
        nrf24l01_device.setRetries(4, 10);
        nrf24l01_device.setAddressWidth(4);
        nrf24l01_device.setPayloadSize(nrf24l01_payload_size);
        nrf24l01_device.openWritingPipe((uint8_t *)"VCU0");
        nrf24l01_device.openReadingPipe(1, (uint8_t *)"HMI0");
        nrf24l01_device.startListening();
      }

      void spinOnce()
      {
        if(fhss_synced == true && since_channel_change >= update_interval)
        {
          // If we're synced, switch channel together with the sender.
          changeChannel();
          since_channel_change -= update_interval;
        }
        if(fhss_synced == false && since_channel_change >= fhss_timeout)
        {
          // If we're not synced, switch channel every 'fhss_timeout', to prevent waiting on channel which is too noisy to
          // receive anything.
          changeChannel();
          since_channel_change -= fhss_timeout;
        }
        while(nrf24l01_device.available())
        {
          uint8_t buffer[nrf24l01_payload_size];
          digitalWrite(LED_BUILTIN, 1);
          nrf24l01_device.read(buffer, nrf24l01_payload_size);
          auto message_processed = false;
          switch(static_cast<RequestMessageType>(buffer[0]))
          {
            case RequestMessageType::Control:
            {
              static_assert(nrf24l01_payload_size >= 7,
                            "the 'control' message requires a payload size of at least 7 bytes");
              int16_t steering = buffer[1] | (buffer[2] << 8 );
              int16_t throttle = buffer[3] | (buffer[4] << 8 );
              int8_t gearbox = buffer[5];
              uint8_t lighting = buffer[6];
              // Sync FHSS with sender.
              fhss_synced = true;
              since_channel_change = 10u;  // Sync FHSS with sender (control message is sent 10ms after channel change).
              // Send message to VCU.
              vcu.handleControlMessage(steering, throttle, gearbox, lighting);
              message_processed = true;
            } break;
            case RequestMessageType::RequestState:
            {
              static_assert(nrf24l01_payload_size >= 2,
                            "the 'request state' message requires a payload size of at least 2 bytes");
              uint8_t state_requested = buffer[1];
              if(state_requested & 0x01)  // Speed and odometer.
              {
                sendSpeedometerMessage();
              }
              if(state_requested & 0x02)  // Orientation.
              {
                sendOrientationMessage();
              }
              if(state_requested & 0x04)  // Location.
              {
                sendLocationMessage();
              }
              if(state_requested & 0x08)  // Altitude.
              {
                sendAltitudeMessage();
              }
            } break;
            default:
            {
              message_processed = false;
            } break;
          }
          if(message_processed == true)
          {
            digitalWrite(LED_BUILTIN, 0);
            since_last_message = 0;
          }
        }
        if(fhss_synced == true && since_last_message > fhss_timeout)
        {
          fhss_synced = false;
        }
      }

    private:

      bool sendSpeedometerMessage()
      {
        static_assert(nrf24l01_payload_size >= 9, "the 'speedometer' message requires a payload size of at least 9 bytes");
        uint8_t buffer[nrf24l01_payload_size];
        // Compose speedometer message.
        buffer[0] = static_cast<uint8_t>(ResponseMessageType::Speedometer);
        auto speed = int16_t(vcu.getSpeed());
        buffer[1] = speed & 0x00ff;
        buffer[2] = (speed & 0xff00) >> 8;
        auto odometer = uint64_t(vcu.getOdometer()) & 0x00ffffff;
        buffer[3] = odometer & 0x0000ff;
        buffer[4] = (odometer & 0x00ff00) >> 8;
        buffer[5] = (odometer & 0xff0000) >> 16;
        auto tripmeter = uint64_t(vcu.getTripmeter()) & 0x00ffffff;
        buffer[6] = tripmeter & 0x0000ff;
        buffer[7] = (tripmeter & 0x00ff00) >> 8;
        buffer[8] = (tripmeter & 0xff0000) >> 16;
        // Transmit speedometer message.
        return sendMessage(buffer);
      }

      bool sendOrientationMessage()
      {
        static_assert(nrf24l01_payload_size >= 8, "the 'orientation' message requires a payload size of at least 8 bytes");
        uint8_t buffer[nrf24l01_payload_size];
        // Compose orientation message.
        buffer[0] = static_cast<uint8_t>(ResponseMessageType::Orientation);
        auto orientation = vcu.getOrientation();
        auto calibration_status = vcu.getImuCalibrationStatus();
        uint16_t yaw = uint16_t(orientation.yaw * 180. / M_PI * 100.);
        buffer[1] = yaw & 0x00ff;
        buffer[2] = (yaw & 0xff00) >> 8;
        int16_t pitch = int16_t(orientation.pitch * 180. / M_PI * 100.);
        buffer[3] = pitch & 0x00ff;
        buffer[4] = (pitch & 0xff00) >> 8;
        int16_t roll =  int16_t(orientation.roll * 180. / M_PI * 100.);
        buffer[5] = roll & 0x00ff;
        buffer[6] = (roll & 0xff00) >> 8;
        buffer[7] = (calibration_status.magnetometer & 0x03) |
                    ((calibration_status.accelerometer & 0x03) << 2) |
                    ((calibration_status.gyroscope & 0x03) << 4) |
                    ((calibration_status.system & 0x03) << 6);
        // Transmit orientation message.
        return sendMessage(buffer);
      }

      bool sendLocationMessage()
      {
        auto gps_data = vcu.getGpsData();
        if(gps_data.valid.location)
        {
          static_assert(nrf24l01_payload_size >= 9, "the 'location' message requires a payload size of at least 9 bytes");
          uint8_t buffer[nrf24l01_payload_size];
          // Compose location message.
          buffer[0] = static_cast<uint8_t>(ResponseMessageType::Location);
          const int32_t latitude = gps_data.latitudeL();
          buffer[1] = latitude & 0x000000ff;
          buffer[2] = (latitude & 0x0000ff00) >> 8;
          buffer[3] = (latitude & 0x00ff0000) >> 16;
          buffer[4] = (latitude & 0xff000000) >> 24;
          const int32_t longitude = gps_data.longitudeL();
          buffer[5] = longitude & 0x000000ff;
          buffer[6] = (longitude & 0x0000ff00) >> 8;
          buffer[7] = (longitude & 0x00ff0000) >> 16;
          buffer[8] = (longitude & 0xff000000) >> 24;
          // Transmit location message.
          return sendMessage(buffer);
        }
        else
        {
          return false;  // Ignore the location request if we don't have a valid location.
        }
        
      }

      bool sendAltitudeMessage()
      {
        auto gps_data = vcu.getGpsData();
        if(gps_data.valid.altitude)
        {
          static_assert(nrf24l01_payload_size >= 5, "the 'altitude' message requires a payload size of at least 5 bytes");
          uint8_t buffer[nrf24l01_payload_size];
          // Compose altitude message.
          buffer[0] = static_cast<uint8_t>(ResponseMessageType::Altitude);
          const int32_t altitude = gps_data.altitude_cm();
          buffer[1] = altitude & 0x000000ff;
          buffer[2] = (altitude & 0x0000ff00) >> 8;
          buffer[3] = (altitude & 0x00ff0000) >> 16;
          buffer[4] = (altitude & 0xff000000) >> 24;
          // Transmit altitude message.
          return sendMessage(buffer);
        }
        else
        {
          return false;  // Ignore the altitude request if we don't have a valid altitude.
        }
      }

      inline bool sendMessage(uint8_t buffer[nrf24l01_payload_size])
      {
        nrf24l01_device.stopListening();
        auto result = nrf24l01_device.write(buffer, nrf24l01_payload_size);
        nrf24l01_device.startListening();
        return result;
      }

      inline void changeChannel()
      {
        nrf24l01_fhss_channel_index = (nrf24l01_fhss_channel_index + 1)
                                      % (sizeof(nrf24l01_fhss_channels) / sizeof(nrf24l01_fhss_channels[0]));
        nrf24l01_device.setChannel(nrf24l01_fhss_channels[nrf24l01_fhss_channel_index]);
      }

  };

}

#endif