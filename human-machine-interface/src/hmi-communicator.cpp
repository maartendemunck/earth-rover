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
      if(control_message_sent && update_sequence_id % 2 == 1)  // Use the odd sequence IDs for configuration.
      {
        if(!car_state.isConfigurationAvailable())
        {
          // If the configuration stored in the VCU is not available, get it first.
          requestNextConfigurationParameter();
        }
        else if(!car_state.isCurrentConfigurationStored())
        {
          // If the configuration is changed on the HMI, send it to the VCU.
          sendNextConfigurationParameter();
        }
        else if(car_state.isConfigurationSaveRequested())
        {
          // If the configuration should be saved, send a 'save configuration' request to the VCU.
          if(sendSaveConfigurationMessage())
          {
            car_state.configurationSaved();
          }
        }
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
        case ResponseMessageType::RadioConfiguration:
        {
          RadioConfigParams stored_configuration {buffer[1], buffer[2]};
          car_state.setStoredRadioConfiguration(stored_configuration);
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
    else if(!car_state.isRadioConfigurationAvailable())
    {
      sendRequestConfigurationMessage(0x80);
    }
  }


  void HmiCommunicator::sendNextConfigurationParameter()
  {
    if(!car_state.isCurrentSteeringConfigurationStored())
    {
      if(sendCurrentSteeringConfiguration())
      {
        car_state.steeringConfigurationStored();
      }
    }
    else if(!car_state.isCurrentThrottleConfigurationStored())
    {
      if(sendCurrentThrottleConfiguration())
      {
        car_state.throttleConfigurationStored();
      }
    }
    else if(!car_state.isCurrentGearboxConfigurationStored())
    {
      if(sendCurrentGearboxConfiguration())
      {
        car_state.gearboxConfigurationStored();
      }
    }
    else if(!car_state.isCurrentRadioConfigurationStored())
    {
      if(sendCurrentRadioConfiguration())
      {
        car_state.radioConfigurationStored();
      }
    }
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
                  "the 'request configuration' message requires a payload size of at least 2 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    buffer[0] = to_integral(RequestMessageType::RequestConfiguration);
    buffer[1] = requested_configuration;
    return sendMessage(buffer);
  }


  bool HmiCommunicator::sendCurrentSteeringConfiguration()
  {
    static_assert(nrf24l01_payload_size >= 8,
                  "the 'send steering configuration' message requires a payload size of at least 8 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    buffer[0] = to_integral(RequestMessageType::ConfigureSteeringServo);
    auto configuration = car_state.getSteeringConfiguration();
    buffer[1] = (configuration.pulse_width_minimum & 0x00ff);
    buffer[2] = (configuration.pulse_width_minimum & 0xff00) >> 8;
    buffer[3] = (configuration.pulse_width_center & 0x00ff);
    buffer[4] = (configuration.pulse_width_center & 0xff00) >> 8;
    buffer[5] = (configuration.pulse_width_maximum & 0x00ff);
    buffer[6] = (configuration.pulse_width_maximum & 0xff00) >> 8;
    buffer[7] = configuration.input_channel;
    return sendMessage(buffer);
  }


  bool HmiCommunicator::sendCurrentThrottleConfiguration()
  {
    static_assert(nrf24l01_payload_size >= 8,
                  "the 'send throttle configuration' message requires a payload size of at least 8 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    buffer[0] = to_integral(RequestMessageType::ConfigureEsc);
    auto configuration = car_state.getThrottleConfiguration();
    buffer[1] = (configuration.pulse_width_minimum & 0x00ff);
    buffer[2] = (configuration.pulse_width_minimum & 0xff00) >> 8;
    buffer[3] = (configuration.pulse_width_center & 0x00ff);
    buffer[4] = (configuration.pulse_width_center & 0xff00) >> 8;
    buffer[5] = (configuration.pulse_width_maximum & 0x00ff);
    buffer[6] = (configuration.pulse_width_maximum & 0xff00) >> 8;
    buffer[7] = configuration.input_channel;
    return sendMessage(buffer);
  }


  bool HmiCommunicator::sendCurrentGearboxConfiguration()
  {
    static_assert(nrf24l01_payload_size >= 9,
                  "the 'send gearbox configuration' message requires a payload size of at least 9 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    buffer[0] = to_integral(RequestMessageType::ConfigureGearboxServo);
    auto configuration = car_state.getGearboxConfiguration();
    buffer[1] = 0x28;  // No reverse gears, has neutral, two forward gears.
    buffer[2] = 0;
    buffer[3] = (configuration.getPulseWidth(0) & 0x00ff);
    buffer[4] = (configuration.getPulseWidth(0) & 0xff00) >> 8;
    buffer[5] = 1;
    buffer[6] = (configuration.getPulseWidth(1) & 0x00ff);
    buffer[7] = (configuration.getPulseWidth(1) & 0xff00) >> 8;
    buffer[8] = configuration.input_channel;
    bool success = sendMessage(buffer);
    buffer[2] = 2;
    buffer[3] = (configuration.getPulseWidth(2) & 0x00ff);
    buffer[4] = (configuration.getPulseWidth(2) & 0xff00) >> 8;
    buffer[5] = 0x80;
    buffer[6] = 0x00;
    buffer[7] = 0x00;
    success &= sendMessage(buffer);
    return success;
  }
  

  bool HmiCommunicator::sendCurrentRadioConfiguration()
  {
    static_assert(nrf24l01_payload_size >= 8,
                  "the 'send thradiorottle configuration' message requires a payload size of at least 3 bytes");
    uint8_t buffer[nrf24l01_payload_size];
    buffer[0] = to_integral(RequestMessageType::ConfigureRadio);
    auto configuration = car_state.getRadioConfiguration();
    buffer[1] = configuration.tx_power;
    buffer[2] = configuration.rx_power;
    return sendMessage(buffer);
  }


  bool HmiCommunicator::sendSaveConfigurationMessage()
  {
    static_assert(nrf24l01_payload_size >= 1,
                  "the 'save configuration' message requires a payload size of at least 1 byte");
    uint8_t buffer[nrf24l01_payload_size];
    buffer[0] = to_integral(RequestMessageType::SaveConfiguration);
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