#ifndef __POSITION_ENCODER__
#define __POSITION_ENCODER__


#include <Arduino.h>
#include <cstdint>
#include "isr-wrapper.hpp"


namespace earth_rover_vcu
{
  template<uint8_t hall_sensor_a_pin, uint8_t hall_sensor_b_pin>
  class PositionEncoder
  {
    private:

      ISRWrapper<hall_sensor_a_pin> isr_wrapper;
      const uint16_t pulses_per_km;
      volatile uint32_t odometer;
      volatile uint32_t tripmeter;
      elapsedMicros since_last_pulse;
      static constexpr uint8_t available_intervals = 8;
      volatile uint32_t between_pulses[available_intervals];

    public:

      PositionEncoder(uint16_t pulses_per_km, uint32_t odometer = 0u, uint32_t tripmeter = 0u):
        pulses_per_km {pulses_per_km},
        odometer {odometer},
        tripmeter {tripmeter},
        between_pulses {0u, 0u, 0u, 0u}
      {
        ;
      }

      ~PositionEncoder() = default;

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

      uint16_t getSpeed() const
      {
        uint32_t numerator = (3600u * 1000000u / pulses_per_km) * 1000u;  // convert elapsedMicros to m/h.
        constexpr uint32_t minimum_interval = 200000;
        uint32_t since_last_pulse_const = since_last_pulse;  // avoid race condition.
        if(since_last_pulse_const > between_pulses[odometer % 4] * 2)
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

      uint32_t getOdometer() const
      {
        return odometer * 1000 / pulses_per_km;
      }

      uint32_t getTripmeter() const
      {
        return tripmeter * 1000 / pulses_per_km;
      }

      void resetTripmeter()
      {
        tripmeter = 0;
      }

    private:

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