//! VCU (vehicle control unit) communicator for the Earth Rover (interface and template implementation).
/*!
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER__VCU_COMMUNICATOR__
#define __EARTH_ROVER__VCU_COMMUNICATOR__


#include <functional>
#include <Arduino.h>
#include <RF24.h>
#undef printf  // RF24.h defined 'printf' as 'Serial.printf', expanding 'Serial.printf' to 'Serial.Serial.printf'.
#include "vcu.hpp"


namespace earth_rover_vcu
{

  //! VCU (vehicle control unit) communicator for the Earth Rover.
  /*!
   *  The VCU communicator handles the communication with the HMI, executing commands and responding to requests.
   * 
   *  \tparam Vcu VCU type.
   * 
   *  \ingroup VCU
   */
  template <typename Vcu>
  class VcuCommunicator
  {
    private:

      //! Request message IDs.
      enum class RequestMessageType: uint8_t
      {
        Control = 0x00, RequestState = 0x10, RequestConfiguration = 0x30,
        ConfigureSteeringServo = 0x20, ConfigureEsc = 0x21, ConfigureGearboxServo = 0x22, ConfigureRadio = 0x24,
        SaveConfiguration = 0x2f
      };
      //! Response message IDs.
      enum class ResponseMessageType: uint8_t
      {
        Speedometer = 0x90, Orientation = 0x91, Location = 0x92, Altitude = 0x93,
        SteeringServoConfiguration = 0xb0, EscConfiguration = 0xb1, GearboxServoConfiguration = 0xb2,
        RadioConfiguration = 0xb4
      };

      //! nRF24L01+ device.
      RF24 nrf24l01_device;
      //! Payload size of the nRF24L01+ messages.
      static constexpr uint8_t nrf24l01_payload_size {9u};
      //! nRF24L01+ channel sequence to use for the FHSS algorithm.
      /*!
       *  This sequence seems to work reliably with the Velleman nRF24L01+ modules.
       */
      const uint8_t nrf24l01_fhss_channels[40]
        {  0,  4,  8, 12, 16, 20, 24, 28, 32, 36,  1,  5,  9, 13, 17, 21, 25, 29, 33, 37,
           2,  6, 10, 14, 18, 22, 26, 30, 34, 38,  3,  7, 11, 15, 19, 23, 27, 31, 35, 39 };
      //! Current channel (as an index to the nrf24l01_fhss_channels array).
      uint8_t nrf24l01_fhss_channel_index;
      //! State update interval.
      static constexpr uint8_t update_interval {50u};
      //! Elapsed time since the last channel change.
      elapsedMillis since_channel_change;
      //! Timeout for the FHSS algorithm (default: the time needed to cycle through all channels).
      uint32_t fhss_timeout {(sizeof(nrf24l01_fhss_channels) / sizeof(nrf24l01_fhss_channels[0])) * update_interval};
      //! True if the FHSS algorithm is synced, false if not.
      bool fhss_synced;
      //! Elapsed time since the last received message.
      elapsedMillis since_last_message;
      //! VCU.
      Vcu & vcu;

    public:

      //! Constructor.
      /*!
       *  \param ce_pin I/O pin used for the nRF24L01+'s CE signal.
       *  \param csn_pin I/O pin used for the nRF24L01+'s CSN signal.
       *  \param vcu VCU.
       */
      VcuCommunicator(uint8_t ce_pin, uint8_t csn_pin, Vcu & vcu)
      :
        nrf24l01_device {ce_pin, csn_pin},
        nrf24l01_fhss_channel_index {0u},
        vcu {vcu}
      {
        ;
      }

      //! Destructor.
      ~VcuCommunicator() = default;

      //! Initialize the VCU communicator.
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

      //! Spinning loop.
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
            case RequestMessageType::RequestConfiguration:
            {
              static_assert(nrf24l01_payload_size >= 2,
                            "the 'request configuration' message requires a payload size of at least 2 bytes");
              uint8_t configuration_requested = buffer[1];
              if(configuration_requested & 0x01)  // Steering servo.
              {
                sendSteeringServoConfigurationMessage();
              }
              if(configuration_requested & 0x02)  // ESC or throttle servo.
              {
                sendEscConfigurationMessage();
              }
              if(configuration_requested & 0x04)  // Gearbox servo.
              {
                sendGearboxServoConfigurationMessage();
              }
              if(configuration_requested & 0x80)  // Radio.
              {
                sendRadioConfigurationMessage();
              }
            } break;
            case RequestMessageType::ConfigureSteeringServo:
            {
              static_assert(nrf24l01_payload_size >= 8,
                            "the 'configure steering servo' message requires a payload size of at least 8 bytes");
              auto pulse_width_left = buffer[1] | (buffer[2] << 8);
              auto pulse_width_center = buffer[3] | (buffer[4] << 8);
              auto pulse_width_right = buffer[5] | (buffer[6] << 8);
              vcu.configureSteeringServo(pulse_width_left, pulse_width_center, pulse_width_right);
              vcu.setSteeringInputChannel(buffer[7]);
            } break;
            case RequestMessageType::ConfigureEsc:
            {
              static_assert(nrf24l01_payload_size >= 8,
                            "the 'configure esc' message requires a payload size of at least 8 bytes");
              auto pulse_width_backwards = buffer[1] | (buffer[2] << 8);
              auto pulse_width_stop = buffer[3] | (buffer[4] << 8);
              auto pulse_width_forward = buffer[5] | (buffer[6] << 8);
              vcu.configureESC(pulse_width_backwards, pulse_width_stop, pulse_width_forward);
              vcu.setThrottleInputChannel(buffer[7]);
            } break;
            case RequestMessageType::ConfigureGearboxServo:
              static_assert(nrf24l01_payload_size >= 8,
                            "the 'configure esc' message requires a payload size of at least 9 bytes");
              for(unsigned index = 0; index < 2; ++ index)
              {
                if(buffer[2 + 3 * index] != 0x80)
                {
                  vcu.configureGearboxServo(
                    buffer[2 + 3 * index], buffer[3 + 3 * index] | (buffer[4 + 3 * index] << 8));
                }
                vcu.setGearboxInputChannel(buffer[8]);
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

      //! Send a speedometer (speed, odometer and tripmeter) response.
      /*!
       *  \return True if the message was successfully sent, false if not.
       */
      bool sendSpeedometerMessage()
      {
        static_assert(nrf24l01_payload_size >= 9,
                      "the 'speedometer' message requires a payload size of at least 9 bytes");
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

      //! Send an orientation (yaw, pitch, roll) response.
      /*!
       *  \return True if the message was successfully sent, false if not.
       */
      bool sendOrientationMessage()
      {
        static_assert(nrf24l01_payload_size >= 8, "the 'orientation' message requires a payload size of at least 8 bytes");
        uint8_t buffer[nrf24l01_payload_size];
        // Compose orientation message.
        buffer[0] = static_cast<uint8_t>(ResponseMessageType::Orientation);
        auto orientation = vcu.getOrientation();
        uint16_t yaw = uint16_t(orientation.yaw * 180. / M_PI * 100.);
        buffer[1] = yaw & 0x00ff;
        buffer[2] = (yaw & 0xff00) >> 8;
        int16_t pitch = int16_t(orientation.pitch * 180. / M_PI * 100.);
        buffer[3] = pitch & 0x00ff;
        buffer[4] = (pitch & 0xff00) >> 8;
        int16_t roll =  int16_t(orientation.roll * 180. / M_PI * 100.);
        buffer[5] = roll & 0x00ff;
        buffer[6] = (roll & 0xff00) >> 8;
        buffer[7] = vcu.isImuFullyCalibrated()? 0xff: 0x00;
        // Transmit orientation message.
        return sendMessage(buffer);
      }

      //! Send an location (latitude and longitude) response.
      /*!
       *  \return True if the message was successfully sent, false if not.
       */
      bool sendLocationMessage()
      {
        auto gps_data = vcu.getGpsData();
        if(gps_data.valid.location)
        {
          static_assert(nrf24l01_payload_size >= 9,
                        "the 'location' message requires a payload size of at least 9 bytes");
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

      //! Send an altitude response.
      /*!
       *  \return True if the message was successfully sent, false if not.
       */
      bool sendAltitudeMessage()
      {
        auto gps_data = vcu.getGpsData();
        if(gps_data.valid.altitude)
        {
          static_assert(nrf24l01_payload_size >= 5,
                        "the 'altitude' message requires a payload size of at least 5 bytes");
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

      //! Send a message with the current configuration of the steering servo.
      /*!
       *  \return True if the message was successfully sent, false if not.
       */
      bool sendSteeringServoConfigurationMessage()
      {
        static_assert(nrf24l01_payload_size >= 8,
                      "the 'steering servo configuration' message requires a payload size of at least 8 bytes");
        uint8_t buffer[nrf24l01_payload_size];
        // Compose the steering servo configuration message.
        buffer[0] = static_cast<uint8_t>(ResponseMessageType::SteeringServoConfiguration);
        auto configuration = vcu.getSteeringServoConfiguration();
        buffer[1] = configuration.pulse_width_left & 0x00ff;
        buffer[2] = (configuration.pulse_width_left & 0xff00) >> 8;
        buffer[3] = configuration.pulse_width_center & 0x00ff;
        buffer[4] = (configuration.pulse_width_center & 0xff00) >> 8;
        buffer[5] = configuration.pulse_width_right & 0x00ff;
        buffer[6] = (configuration.pulse_width_right & 0xff00) >> 8;
        buffer[7] = vcu.getSteeringInputChannel();
        // Transmit the steering servo configuration message.
        return sendMessage(buffer);
      }

      //! Send a message with the current configuration of the ESC or throttle servo.
      /*!
       *  \return True if the message was successfully sent, false if not.
       */
      bool sendEscConfigurationMessage()
      {
        static_assert(nrf24l01_payload_size >= 8,
                      "the 'ESC servo configuration' message requires a payload size of at least 8 bytes");
        uint8_t buffer[nrf24l01_payload_size];
        // Compose the ESC configuration message.
        buffer[0] = static_cast<uint8_t>(ResponseMessageType::EscConfiguration);
        auto configuration = vcu.getPowertrainConfiguration();
        buffer[1] = configuration.esc.pulse_width_reverse & 0x00ff;
        buffer[2] = (configuration.esc.pulse_width_reverse & 0xff00) >> 8;
        buffer[3] = configuration.esc.pulse_width_stop & 0x00ff;
        buffer[4] = (configuration.esc.pulse_width_stop & 0xff00) >> 8;
        buffer[5] = configuration.esc.pulse_width_forward & 0x00ff;
        buffer[6] = (configuration.esc.pulse_width_forward & 0xff00) >> 8;
        buffer[7] = vcu.getThrottleInputChannel();
        // Transmit the ESC configuration message.
        return sendMessage(buffer);
      }

      //! Send (a) message(s) with the current configuration of the gearbox servo.
      /*!
       *  \return True if the message(s) was/were successfully sent, false if not.
       */
      bool sendGearboxServoConfigurationMessage()
      {
        static_assert(nrf24l01_payload_size >= 8,
                      "the 'gearbox servo configuration' message requires a payload size of at least 9 bytes");
        uint8_t buffer[nrf24l01_payload_size];
        // Compose and transmit the gearbox servo configuration message(s).
        bool success = true;
        buffer[0] = static_cast<uint8_t>(ResponseMessageType::GearboxServoConfiguration);
        auto configuration = vcu.getPowertrainConfiguration();
        buffer[1] = 0x28;  // No reverse gears, has neutral, two forward gears.
        buffer[2] = 0;
        buffer[3] = configuration.gearbox.pulse_width_neutral & 0x00ff;
        buffer[4] = (configuration.gearbox.pulse_width_neutral & 0xff00) >> 8;
        buffer[5] = 1;
        buffer[6] = configuration.gearbox.pulse_width_low & 0x00ff;
        buffer[7] = (configuration.gearbox.pulse_width_low & 0xff00) >> 8;
        buffer[8] = vcu.getGearboxInputChannel();
        success &= sendMessage(buffer);
        buffer[2] = 2;
        buffer[3] = configuration.gearbox.pulse_width_high & 0x00ff;
        buffer[4] = (configuration.gearbox.pulse_width_high & 0xff00) >> 8;
        buffer[5] = 0x80;  // Unused
        buffer[6] = 0x00;
        buffer[7] = 0x00;
        success &= sendMessage(buffer);
        // Return the status.
        return success;
      }

      //! Send a message with the current configuration of the radios.
      /*!
       *  \return True if the message was successfully sent, false if not.
       */
      bool sendRadioConfigurationMessage()
      {
        static_assert(nrf24l01_payload_size >= 8,
                      "the 'radio configuration' message requires a payload size of at least 3 bytes");
        uint8_t buffer[nrf24l01_payload_size];
        buffer[0] = static_cast<uint8_t>(ResponseMessageType::RadioConfiguration);
        buffer[1] = vcu.getHmiRadioPowerLevel();
        buffer[2] = vcu.getVcuRadioPowerLevel();
        // Transmit radio configuration message.
        return sendMessage(buffer);
      }

      //! Send a message via the nRF24L01+ transceiver.
      /*!
       *  \param buffer Message (of nrf24l01_payload_size bytes).
       *  \return True if the message was successfully sent, false if not.
       */
      inline bool sendMessage(uint8_t buffer[nrf24l01_payload_size])
      {
        nrf24l01_device.stopListening();
        auto result = nrf24l01_device.write(buffer, nrf24l01_payload_size);
        nrf24l01_device.startListening();
        return result;
      }

      //! Hop to the next channel in the list.
      inline void changeChannel()
      {
        nrf24l01_fhss_channel_index = (nrf24l01_fhss_channel_index + 1)
                                      % (sizeof(nrf24l01_fhss_channels) / sizeof(nrf24l01_fhss_channels[0]));
        nrf24l01_device.setChannel(nrf24l01_fhss_channels[nrf24l01_fhss_channel_index]);
      }

  };

}

#endif