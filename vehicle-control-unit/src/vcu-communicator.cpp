#include "vcu-communicator.hpp"


namespace earth_rover
{

  constexpr uint8_t VcuCommunicator::nrf24l01_fhss_channels[];  // Initialised in header file.


  VcuCommunicator::VcuCommunicator(uint8_t ce_pin, uint8_t csn_pin, Vcu & vcu):
    nrf24l01_device {ce_pin, csn_pin},
    nrf24l01_fhss_channel_index {0u},
    vcu {vcu}
  {
    ;
  }


  void
  VcuCommunicator::setup()
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


  void
  VcuCommunicator::spinOnce()
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
      switch(static_cast<MessageType>(buffer[0]))
      {
        case MessageType::Control:
        {
          static_assert(nrf24l01_payload_size >= 7, "control message requires a payload size of at least 7 bytes");
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


  void VcuCommunicator::changeChannel()
  {
    nrf24l01_fhss_channel_index = (nrf24l01_fhss_channel_index + 1)
                                  % (sizeof(nrf24l01_fhss_channels) / sizeof(nrf24l01_fhss_channels[0]));
    nrf24l01_device.setChannel(nrf24l01_fhss_channels[nrf24l01_fhss_channel_index]);
  }

}