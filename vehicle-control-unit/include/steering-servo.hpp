//! Steering servo device driver for the Earth Rover's VCU (interface and inline part of the implementation).
/*!
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_VCU__STEERING_SERVO__
#define __EARTH_ROVER_VCU__STEERING_SERVO__


#include <cstdint>
#include "configured-servo.hpp"


namespace earth_rover_vcu
{

  //! Steering servo device driver for the Earth Rover's VCU.
  /*!
   *  This class abstracts the steering device used in the Earth Rover.
   * 
   *  \ingroup VCU
   */
  class SteeringServo
  {
    public:

      //! Steering servo configuration.
      struct Configuration
      {
        uint16_t pulse_width_left;    //!< Pulse width to steer to the left.
        uint16_t pulse_width_center;  //!< Pulse width to drive straight.
        uint16_t pulse_width_right;   //!< Pulse width to steer to the right.
      };

    private:

      uint8_t pin_number;              //!< I/O pin used to control the steering servo.
      ConfiguredServo steering_servo;  //!< Steering servo controller.
      int16_t current_steering_angle;  //!< Current (normalized) steering angle (-1000...0...+1000).

    public:

      //! Constructor.
      /*!
       *  \param pin_number Output pin used to control the steering servo.
       */
      SteeringServo(uint8_t pin_number);

      //! Default destructor.
      ~SteeringServo() = default;

      //! Initialize the steering servo.
      inline void setup()
      {
        current_steering_angle = 0;
        steering_servo.setup();
        steering_servo.setPosition(current_steering_angle);
      }

      //! Spinning loop.
      /*!
       *  \internal
       *  The steering servo's spinning loop does nothing.
       */
      inline void spinOnce()
      {
        ;
      }

      //! Set the (normalized) steering angle.
      /*!
       *  \param steering_angle Normalized (-1000 = full left, 0 = center, +1000 = full right) steering angle.
       */
      inline void setNormalizedSteeringAngle(int16_t steering_angle)
      {
        steering_servo.setPosition(steering_angle);
        current_steering_angle = steering_angle;
      }

      //! Configure the steering servo.
      /*!
       *  \param pulse_width_left Pulse width to maximally steer to the left.
       *  \param pulse_width_center Pulse width to steer straight.
       *  \param pulse_width_right Pulse width to maximally steer to the right.
       */
      void configureSteeringServo(uint16_t pulse_width_left, uint16_t pulse_width_center, uint16_t pulse_width_right);

      //! Get the current configuration.
      /*!
       *  \return The current configuration of the steering servo.
       */
      Configuration getConfiguration()
      {
        auto configuration = steering_servo.getConfiguration();
        return Configuration
          {configuration.minimum_pulse_width, configuration.center_pulse_width, configuration.maximum_pulse_width};
      }

      //! The (minimal) size of the configuration block.
      static constexpr uint16_t configuration_size = 7u;

      //! Save the configuration to a buffer.
      /*!
       *  The buffer should be at least getConfigurationSize bytes. If not, no configuration is written.
       * 
       *  \param data Pointer to the buffer.
       *  \param size Size of the buffer.
       *  \return true if the configuration is written; false if not (if the buffer isn't large enough).
       */
      bool saveConfiguration(uint8_t * data, uint16_t size);

      //! Load the configuration from a buffer.
      /*!
       *  The buffer should be at least getConfigurationSize bytes. If not, no configuration is read or applied.
       * 
       *  \param data Pointer to the buffer.
       *  \param size Size of the buffer.
       *  \return true if the configuration is applied; false if not (buffer not large enough or checksum wrong).
       */
      bool loadConfiguration(uint8_t * data, uint16_t size);

  };

}

#endif