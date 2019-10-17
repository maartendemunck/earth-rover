//! HMI (human machine interface) communicator for the Earth Rover (implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#include "hmi-communicator.hpp"
#include "from-to-integral.hpp"


namespace earth_rover_hmi
{

  constexpr uint8_t HmiCommunicator::nrf24l01_fhss_channels[];  // Initialised in header file.


  HmiCommunicator::HmiCommunicator(uint8_t ce_pin, uint8_t csn_pin, CarState & car_state):
    nrf24l01_device {ce_pin, csn_pin},
    nrf24l01_fhss_channel_index {0u},
    update_sequence_id {0u},
    car_state {car_state}
  {
    ;
  }


  void HmiCommunicator::setup()
  {
    // Setup nRF24L01+.
    nrf24l01_device.begin();
    nrf24l01_device.setPALevel(RF24_PA_LOW);
    nrf24l01_device.setDataRate(RF24_250KBPS);
    nrf24l01_device.setChannel(nrf24l01_fhss_channels[nrf24l01_fhss_channel_index]);
    nrf24l01_device.setRetries(4, 10);
    nrf24l01_device.setAddressWidth(4);
    nrf24l01_device.setPayloadSize(nrf24l01_payload_size);
    nrf24l01_device.openWritingPipe((uint8_t *)"HMI0");
    nrf24l01_device.openReadingPipe(1, (uint8_t *)"VCU0");
    nrf24l01_device.startListening();
  }


  void HmiCommunicator::spinOnce()
  {
    if(channel_changed == false && since_update >= update_interval - 10u)
    {
      changeChannel();
      channel_changed = true;
    }
    else if(channel_changed == true && since_update >= update_interval)
    {
      bool control_message_sent = sendControlMessage();
      since_update -= update_interval;
      if(control_message_sent && update_sequence_id % 6 == 0)
      {
        sendRequestStateMessage(0x01);
      }
      if(control_message_sent && update_sequence_id % 6 == 2)
      {
        sendRequestStateMessage(0x02);
      }
      if(control_message_sent && update_sequence_id % 12 == 4)
      {
        sendRequestStateMessage(0x04);
      }
      if(control_message_sent && update_sequence_id % 12 == 10)
      {
        sendRequestStateMessage(0x08);
      }
      if(!car_state.isConfigurationAvailable() && control_message_sent && update_sequence_id % 2 == 1)
      {
        requestNextConfigurationParameter();
      }
      ++ update_sequence_id;
      channel_changed = false;
    }
    while(nrf24l01_device.available())
    {
      uint8_t buffer[nrf24l01_payload_size];
      nrf24l01_device.read(buffer, nrf24l01_payload_size);
      switch(static_cast<ResponseMessageType>(buffer[0]))
      {
        case ResponseMessageType::Speedometer:
        {
          CarState::Speedometer speedometer;
          speedometer.speed = float(uint16_t(buffer[1] | (buffer[2] << 8))) / 1000.;
          speedometer.odometer = float(uint32_t(buffer[3] | (buffer[4] << 8) | (buffer[5] << 16))) / 1000.;
          speedometer.tripmeter = float(uint32_t(buffer[6] | (buffer[7] << 8) | (buffer[8] << 16))) / 1000.;
          car_state.setSpeedometer(speedometer);
        } break;
        case ResponseMessageType::Orientation:
        {
          CarState::Orientation orientation;
          orientation.yaw = float(uint16_t(buffer[1] | (buffer[2] << 8))) / 100.;
          orientation.pitch = float(int16_t(buffer[3] | (buffer[4] << 8))) / 100.;
          orientation.roll = float(int16_t(buffer[5] | (buffer[6] << 8))) / 100.;
          car_state.setOrientation(orientation);
        } break;
        case ResponseMessageType::Location:
        {
          CarState::Location location;
          location.latitude.From(int32_t(buffer[1]) | (int32_t(buffer[2]) << 8)
                                 | (int32_t(buffer[3]) << 16) | (int32_t(buffer[4]) << 24));
          location.longitude.From(int32_t(buffer[5]) | (int32_t(buffer[6]) << 8)
                                  | (int32_t(buffer[7]) << 16) | (int32_t(buffer[8]) << 24));
          car_state.setLocation(location, true);
        } break;
        case ResponseMessageType::Altitude:
        {
          car_state.setAltitude(int32_t(buffer[1] | (buffer[2] << 8)
                                | (buffer[3] << 16) | (buffer[4] << 24)), true);
        } break;
        case ResponseMessageType::SteeringServoConfiguration:
        {
          ServoConfigParams stored_configuration
          {
            uint8_t(buffer[7]),                      // Input channel.
            uint16_t(buffer[1] | (buffer[2] << 8)),  // Pulse width to steer left.
            uint16_t(buffer[3] | (buffer[4] << 8)),  // Pulse width to drive straight ahead.
            uint16_t(buffer[5] | (buffer[6] << 8)),  // Pulse width to steer right.
            true                                     // Enforce pulse width limits.
          };
          car_state.setStoredSteeringConfiguration(stored_configuration);
        } break;
        case ResponseMessageType::EscConfiguration:
        {
          ServoConfigParams stored_configuration
          {
            uint8_t(buffer[7]),                      // Input channel.
            uint16_t(buffer[1] | (buffer[2] << 8)),  // Pulse width to drive backwards.
            uint16_t(buffer[3] | (buffer[4] << 8)),  // Pulse width to stop.
            uint16_t(buffer[5] | (buffer[6] << 8)),  // Pulse width to drive forward.
            true                                     // Enforce pulse width limits.
          };
          car_state.setStoredThrottleConfiguration(stored_configuration);
        } break;
        case ResponseMessageType::GearboxServoConfiguration:
        {
          auto stored_configuration = car_state.getGearboxConfiguration();
          stored_configuration.input_channel = uint8_t(buffer[8]);
          for(unsigned index = 0; index < 2; ++ index)
          {
            int8_t gear = int8_t(buffer[2 + index * 3]);
            uint16_t pulse_width = uint16_t(buffer[3 + index * 3] | (buffer[4 + index * 3] << 8));
            if(gear >= 0 && gear <= 2)
            {
              stored_configuration.setPulseWidth(gear, pulse_width);
            }
          }
          bool complete = true;
          for(unsigned index = 0; index <= 2; ++ index)
          {
            if(stored_configuration.getPulseWidth(index) == 0 || stored_configuration.getPulseWidth(index) == 0xffff)
            {
              complete = false;
            }
          }
          car_state.setStoredGearboxConfiguration(stored_configuration, complete);
        } break;
      }
    }
  }


  void HmiCommunicator::requestNextConfigurationParameter()
  {
    if(!car_state.isSteeringConfigurationAvailable())
    {
      sendRequestConfigurationMessage(0x01);
    }
    else if(!car_state.isThrottleConfigurationAvailable())
    {
      sendRequestConfigurationMessage(0x02);
    }
    else if(!car_state.isGearboxConfigurationAvailable())
    {
      sendRequestConfigurationMessage(0x04);
    }
    /*
    else if(!car_state.isRadioConfigurationAvailable())
    {
      sendRequestConfigurationMessage(0x80);
    }
    */
  }

  bool HmiCommunicator::sendControlMessage()
  {
    static_assert(nrf24l01_payload_size >= 5, "control message requires a payload size of at least 5 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    auto steering = car_state.getCurrentSteeringPosition();
    auto throttle = car_state.getCurrentThrottlePosition();
    auto gear = car_state.getCurrentGear();
    auto lighting = car_state.getRequestedLightingState();
    buffer[0] = to_integral(RequestMessageType::Control);
    buffer[1] = (steering) & 0xff;
    buffer[2] = (steering >> 8) & 0xff;
    buffer[3] = (throttle) & 0xff;
    buffer[4] = (throttle >> 8) & 0xff;
    buffer[5] = gear;
    buffer[6] = lighting.turn_signal_right
                | (lighting.turn_signal_left << 1)
                | (lighting.dipped_beam << 2)
                | (lighting.high_beam << 3)
                | (lighting.hazard_flashers << 4);
    return sendMessage(buffer);
  }


  bool HmiCommunicator::sendRequestStateMessage(uint8_t requested_state)
  {
    static_assert(nrf24l01_payload_size >= 2,
                  "the 'request state' message requires a payload size of at least 2 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    buffer[0] = to_integral(RequestMessageType::RequestState);
    buffer[1] = requested_state;
    return sendMessage(buffer);
  }


  bool HmiCommunicator::sendRequestConfigurationMessage(uint8_t requested_configuration)
  {
    static_assert(nrf24l01_payload_size >= 2,
                  "the 'request state' message requires a payload size of at least 2 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    buffer[0] = to_integral(RequestMessageType::RequestConfiguration);
    buffer[1] = requested_configuration;
    return sendMessage(buffer);
  }


  bool HmiCommunicator::sendMessage(uint8_t buffer[nrf24l01_payload_size])
  {
    digitalWrite(LED_BUILTIN, 1);
    nrf24l01_device.stopListening();
    auto result = nrf24l01_device.write(buffer, nrf24l01_payload_size);
    nrf24l01_device.startListening();
    if(result)
    {
      digitalWrite(LED_BUILTIN, 0);
    }
    
    return result;
  }


  void HmiCommunicator::changeChannel()
  {
    nrf24l01_fhss_channel_index = (nrf24l01_fhss_channel_index + 1)
                                  % (sizeof(nrf24l01_fhss_channels) / sizeof(nrf24l01_fhss_channels[0]));
    nrf24l01_device.setChannel(nrf24l01_fhss_channels[nrf24l01_fhss_channel_index]);
  }

}