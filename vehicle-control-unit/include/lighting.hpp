//! Automotive lighting device driver for the Earth Rover's VCU (interface and inline part of the implementation).
/*!
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_VCU__LIGHTING__
#define __EARTH_ROVER_VCU__LIGHTING__


#include <cstdint>
#include <Arduino.h>


namespace earth_rover_vcu
{

  //! Automotive lighting device driver for the Earth Rover's VCU.
  /*!
   *  This class abstracts the automotive lighting used in the Earth Rover.
   * 
   *  \ingroup VCU
   */
  class Lighting
  {

    private:

      uint8_t head_lamp_pin_number;  //!< I/O pin used to control the head lamps.
      uint8_t tail_lamp_pin_number;  //!< I/O pin used to control the tail lamps.
      uint8_t turn_signal_right_pin_number;  //!< I/O pin used to control the right turn signal.
      uint8_t turn_signal_left_pin_number;  //!< I/O pin used to control the left turn signal.

      bool turn_signal_right_state;  //!< Current state of the right turn signal.
      bool turn_signal_left_state;  //!< Current state of the left turn signal.
      bool dipped_beam_state;  //!< Current state of the dipped beam headlamps.
      bool high_beam_state;  //!< Current state of the high beam headlamps.
      bool hazard_flashers_state;  //!< Current state of the hazard flashers.
      bool stop_lamps_state;  //!< Current state of the stop lamps.

      static constexpr uint16_t turn_signal_period = 750u;  //!< Period of the turn signal (in ms).
      elapsedMillis since_turn_signal_flipped;  //!< Time (in ms) since the turn signals flipped state.
      bool turn_signal_flipped_state;  //!< Current state (on/off) of the turn signals or hazard flashers.

    public:

      //! Constructor
      /*!
       *  \param head_lamp_pin_number I/O pin used to control the head lamps.
       *  \param tail_lamp_pin_number I/O pin used to control the tail lamps.
       *  \param turn_signal_right_pin_number I/O pin used to control the right turn signal.
       *  \param turn_signal_left_pin_number I/O pin used to control the left turn signal.
       */
      Lighting(
        uint8_t head_lamp_pin_number, uint8_t tail_lamp_pin_number,
        uint8_t turn_signal_right_pin_number, uint8_t turn_signal_left_pin_number);

      //! Destructor.
      ~Lighting();

      //! Initialize the automotive lighting.
      void setup();

      //! Spinning loop.
      /*!
       *  \internal
       *  The spinning loop controls the turn signals and the hazard flashers.
       */
      void spinOnce();

      //! Set the state of the right turn signal.
      /*!
       *  \param state New state of the right turn signal.
       */
      void setTurnSignalRight(bool state);

      //! Set the state of the left turn signal.
      /*!
       *  \param state New state of the left turn signal.
       */
      void setTurnSignalLeft(bool state);

      //! Set the state of the dipped beam headlamps.
      /*!
       *  \param state New state of the dipped beam headlamps.
       */
      inline void setDippedBeam(bool state)
      {
        dipped_beam_state = state;
        updateSteadyLighting();
      }

      //! Set the state of the high beam headlamps.
      /*!
       *  \param state New state of the high beam headlamps.
       */
      inline void setHighBeam(bool state)
      {
        high_beam_state = state;
        updateSteadyLighting();
      }
      
      //! Set the state of the stop lamps.
      /*!
       *  \param state New state of the stop lamps.
       */
      inline void setStopLamps(bool state)
      {
        stop_lamps_state = state;
        updateSteadyLighting();
      }

      //! Set the state of the hazard flashers.
      /*!
       *  \param state New state of the hazard flashers.
       */
      void setHazardFlashers(bool state);

      //! Get the (minimal) size of the configuration block.
      /*!
       *  The automotive lighting driver has no runtime configuration settings. This function is provided for
       *  compatibility with other car device drivers.
       *
       *  \return The minimal size of the configuration block.
       */
      inline uint16_t getConfigurationSize()
      {
        return 0u;
      }

      //! Save the configuration to a buffer.
      /*!
       *  The automotive lighting driver has no runtime configuration settings. This function is provided for
       *  compatibility with other car device drivers.
       * 
       *  \param data Pointer to the buffer.
       *  \param size Size of the buffer.
       *  \return true if the configuration is written; false if not (if the buffer isn't large enough).
       */
      inline bool saveConfiguration(uint8_t * data, uint16_t size)
      {
        return true;
      }

      //! Load the configuration from a buffer.
      /*!
       *  The automotive lighting driver has no runtime configuration settings. This function is provided for
       *  compatibility with other car device drivers.
       * 
       *  \param data Pointer to the buffer.
       *  \param size Size of the buffer.
       *  \return true if the configuration is applied; false if not (buffer not large enough or checksum wrong).
       */
      inline bool loadConfiguration(uint8_t * data, uint16_t size)
      {
        return true;
      }

    private:

      //! Update the state of the steady lighting (head lamps, tail lamps and stop lamps).
      void updateSteadyLighting();

  };

}

#endif