//! HMI (human machine interface) communicator for the Earth Rover (interface and inline part of the implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_HMI__HMI_COMMUNICATOR__
#define __EARTH_ROVER_HMI__HMI_COMMUNICATOR__


#include <cstdint>
#include <RF24.h>
#undef printf  // RF24.h defined 'printf' as 'Serial.printf', expanding 'Serial.printf' to 'Serial.Serial.printf'.
#include "car-configuration.hpp"
#include "car-state.hpp"


namespace earth_rover_hmi
{

  //! HMI (human machine interface) communicator for the Earth Rover.
  /*!
   *  The HMI communicator handles the communication with the VCU, sending commands and interpreting responses.
   *
   *  \ingroup HMI
   */
  class HmiCommunicator
  {
    private:

      //! Request message IDs.
      enum class RequestMessageType: uint8_t
        { Control = 0x00, RequestState = 0x10 };
      //! Response message IDs.
      enum class ResponseMessageType: uint8_t
        { Speedometer = 0x90, Orientation = 0x91, Location = 0x92, Altitude = 0x93 };

      //! nRF24L01+ device.
      RF24 nrf24l01_device;
      //! Payload size of the nRF24L01+ packahes.
      static constexpr uint8_t nrf24l01_payload_size {9u};
      //! FHSS channel sequence.
      /*!
       *  This sequence seems to work reliably with the Velleman nRF24L01+ modules.
       */
      static constexpr uint8_t nrf24l01_fhss_channels[]
        {  0,  4,  8, 12, 16, 20, 24, 28, 32, 36,  1,  5,  9, 13, 17, 21, 25, 29, 33, 37,
           2,  6, 10, 14, 18, 22, 26, 30, 34, 38,  3,  7, 11, 15, 19, 23, 27, 31, 35, 39 };
      //! Current channel as an index to the FHSS channel sequence.
      uint8_t nrf24l01_fhss_channel_index;
      //! FHSS and request interval (in ms).
      static constexpr uint8_t update_interval {50u};
      //! Elapsed time since the last control message.
      elapsedMillis since_update;
      //! Update sequence ID.
      /*!
       *  The update sequence ID defines which information is requested from the VCU.
       */
      uint8_t update_sequence_id;
      //! Flag to check whether the channel changed in this update interval.
      bool channel_changed;

      //! Car configuration.
      CarConfiguration & car_configuration;
      //! Car state (digital twin).
      CarState & car_state;

    public:

      //! Constructor.
      /*!
       *  \param ce_pin I/O pin used for the nRF24L01+'s CE signal.
       *  \param csn_pin I/O pin used for the nRF24L01+'s CSN signal.
       *  \param car_configuration Car configuration.
       *  \param car_state Car state (digital twin).
       */
      HmiCommunicator(uint8_t ce_pin, uint8_t csn_pin, CarConfiguration & car_configuration, CarState & car_state);

      //! Default destructor.
      ~HmiCommunicator() = default;

      //! Initialize the HMI communicator.
      void setup();

      //! Spinning loop.
      void spinOnce();

    private:

      //! Send a control message to the VCU.
      /*!
       *  Send a control message with the current car state.
       * 
       *  \return True if the message was sent successfully, false if not.
       */
      bool sendControlMessage();

      //! Send a request state message to the VCU.
      /*!
       *  \param requested_state Requested state (bit 0: speedometer, odometer and trip meter, bit 1: orientation,
       *                                          bit 3: GPS location, bit 4: GPS altitude).
       *  \return True if the message was sent successfully, false if not.
       */
      bool sendRequestStateMessage(uint8_t requested_state);

      //! Send a message to the VCU.
      /*!
       *  \param buffer Message.
       *  \return True if the message was sent successfully, false if not.
       */
      bool sendMessage(uint8_t buffer[nrf24l01_payload_size]);

      //! Switch to the next channel in the FHSS channel list.
      void changeChannel();

  };

}

#endif