//! Position encoder device driver for the Earth Rover's VCU (interface and template implementation).
/*!
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_VCU__POSITION_ENCODER__
#define __EARTH_ROVER_VCU__POSITION_ENCODER__


#include <Arduino.h>
#include <cstdint>
#include "isr-wrapper.hpp"


namespace earth_rover_vcu
{

  //! Position encoder device driver for the Earth Rover's VCU.
  /*!
   *  This class abstracts the position encoder used in the Earth Rover.
   * 
   *  \tparam hall_sensor_a_pin I/O pin used for the first Hall effect sensor.
   *  \tparam hall_sensor_b_pin I/O pin used for the second Hall effect sensor.
   * 
   *  \ingroup VCU
   */
  template<uint8_t hall_sensor_a_pin, uint8_t hall_sensor_b_pin>
  class PositionEncoder
  {
    private:

      ISRWrapper<digitalPinToInterrupt(hall_sensor_a_pin)> isr_wrapper;  //!< ISR for the first Hall effect sensor.
      const uint16_t pulses_per_km;                           //!< Number of pulses per km.
      volatile uint32_t odometer;                             //!< Current value of the odometer (in pulses).
      uint32_t odometer_eeprom;                               //!< Value of the odometer saved in EEPROM (in pulses).
      volatile uint32_t tripmeter;                            //!< Current value of the trip meter (in pulses).
      elapsedMicros since_last_pulse;                         //!< Elapsed time since the last pulse.
      static constexpr uint8_t available_intervals = 8;       //!< Number of between-pulse-intervals stored.
      volatile uint32_t between_pulses[available_intervals];  //!< Stored between-pulse-intervals.

      static constexpr uint16_t serialized_resolution {16};   //!< Resolution of serialized odometer value.

    public:

      //! Constructor.
      /*!
       *  \param pulses_per_km Encoder pulses per km.
       *  \param odometer Initial value (pulses) of the odometer.
       *  \param tripmeter Initial value (pulses) of the tripmeter.
       */
      PositionEncoder(uint16_t pulses_per_km, uint32_t odometer = 0u, uint32_t tripmeter = 0u)
      :
        pulses_per_km {pulses_per_km},
        odometer {odometer},
        odometer_eeprom {odometer},
        tripmeter {tripmeter}
      {
        ;
      }

      //! Default destructor.
      ~PositionEncoder() = default;

      //! Initialize the position encoder.
      void setup()
      {
        pinMode(hall_sensor_a_pin, INPUT);
        pinMode(hall_sensor_b_pin, INPUT);
        if(!isr_wrapper.attachISR(
              std::bind(&PositionEncoder<hall_sensor_a_pin, hall_sensor_b_pin>::interruptHandler, this),
              FALLING))
        {
          // Unable to attach ISR handler.
        }
      }

      //! Spinning loop.
      /*!
       *  \internal
       *  The position encoder's spinning loop does nothing.
       */
      void spinOnce()
      {
        ;
      }

      //! Get the current speed of the Earth Rover.
      /*!
       *  \return The current speed of the earth rover in m/h (1e-3 km/h).
       */
      uint16_t getSpeed() const
      {
        uint32_t numerator = (3600u * 1000000u / pulses_per_km) * 1000u;  // convert elapsedMicros to m/h.
        constexpr uint32_t minimum_interval = 200000;
        uint32_t since_last_pulse_const = since_last_pulse;  // avoid race condition.
        if(since_last_pulse_const > between_pulses[odometer % available_intervals] * 2)
        {
          return int16_t(numerator / since_last_pulse_const);
        }
        else
        {
          uint32_t measured_interval = 0;
          int8_t measured_pulses = 0;
          while(measured_pulses < available_intervals && measured_interval < minimum_interval)
          {
            measured_interval += between_pulses[(odometer - measured_pulses) % available_intervals];
            ++ measured_pulses;
          }
          return int16_t(numerator / (measured_interval / measured_pulses));
        }
      }

      //! Get the current value of the odometer in m.
      /*!
       *  \return The current value of the odometer in m.
       */
      uint32_t getOdometer() const
      {
        return odometer * 1000 / pulses_per_km;
      }

      //! Get the current value of the trip meter in m.
      /*!
       *  \return The current value of the trip meter in m.
       */
      uint32_t getTripmeter() const
      {
        return tripmeter * 1000 / pulses_per_km;
      }

      //! Reset the trip meter.
      void resetTripmeter()
      {
        tripmeter = 0;
      }

      //! Check whether the odometer record in EEPROM should be updated.
      /*!
       *  \return True if the odometer record in EEPROM should be updated.
       */
      bool saveRequired()
      {
        return (odometer - odometer_eeprom) > 16u;
      }

      //! Serialize the current value of the odometer.
      /*!
       *  This function uses the technique described [here](https://arduino.stackexchange.com/questions/16301/)
       *  using (size - 2) counters and adds a 16 bit (uint16) offset.
       *
       *  \param buffer Output buffer.
       *  \param size Size of the buffer.
       *  \return True to write a new record, false to update the existing record (or if the serialization failed).
       */
      bool serialize(uint8_t * buffer, uint16_t size)
      {
        // Check size of the serialized record to prevent buffer overflows.
        if(size < 3)
        {
          return false;
        }
        uint16_t counter_range = ((size - 2u) * UINT8_MAX + 1);
        // Calculate values.
        uint32_t value = odometer / serialized_resolution;
        uint16_t offset = value / counter_range;
        uint16_t counter = value % counter_range;
        uint8_t base = counter / (size - 2u);
        uint8_t augmented = counter % (size - 2u);
        // Check whether we'd prefer to update the existing record or to write a new one.
        bool write_new_record = false;
        if(offset != (buffer[0] | (buffer[1] << 8)))
        {
          write_new_record = true;
          buffer[0] = offset & 0x00ff;
          buffer[1] = (offset & 0xff00) >> 8;
        }
        // Compose record.
        for(uint8_t index = 0; index < size - 2u; ++ index)
        {
          buffer[2 + index] = index < augmented? base + 1: base;
        }
        odometer_eeprom = value * serialized_resolution;
        return write_new_record;
      }

      //! Read the value of the odometer from a serialized record.
      /*!
       *  \param buffer Output buffer.
       *  \param size Size of the output buffer.
       *  \return True if the deserialization was successful, false if it failed.
       */
      bool deserialize(uint8_t * buffer, uint16_t size)
      {
        // Check size of the serialized record to prevent buffer overflows.
        if(size < 3)
        {
          return false;
        }
        // Restore odometer value and set odometer.
        uint16_t offset = buffer[0] | (buffer[1] << 8);
        uint8_t base = buffer[size - 1];
        uint8_t augmented = 0;
        while(buffer[2 + augmented] > base)
        {
          ++ augmented;
        }
        uint32_t value = offset * ((size - 2u) * UINT8_MAX + 1) + (base * (size - 2u)) + augmented;
        odometer_eeprom = value * serialized_resolution;
        odometer = odometer_eeprom;
        // Odometer successfully restored.
        return true;
      }

    private:

      //! Interrupt handler for the falling edge of the first Hall effect sensor.
      void interruptHandler()
      {
        ++ odometer;
        ++ tripmeter;
        between_pulses[odometer % available_intervals] = since_last_pulse;
        since_last_pulse -= between_pulses[odometer % available_intervals];
        /*
        if(digitalRead(hall_sensor_b_pin) == LOW)
        { 
          // Driving forward.
        }
        else
        { 
          // Driving backward.
        }
        */
      }

  };

}

#endif