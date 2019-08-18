#ifndef __CONFIGURED_SERVO__
#define __CONFIGURED_SERVO__


#include <Servo.h>


namespace earth_rover
{

  class ConfiguredServo
  {

    public:

      struct Configuration {
        uint8_t pin_number;
        uint16_t minimum_pulse_width;
        uint16_t maximum_pulse_width;
        uint16_t center_pulse_width;
        uint16_t initial_pulse_width;
        bool enforce_pulse_width_limits;
      };

    private:

      static constexpr uint16_t default_minimum_pulse_width {1000U};
      static constexpr uint16_t default_maximum_pulse_width {2000U};
      static constexpr uint16_t default_center_pulse_width {1500U};
      static constexpr uint16_t default_initial_pulse_width {1500U};
      static constexpr bool default_enforce_pulse_width_limits {true};

      Servo servo;
      Configuration configuration;
      uint16_t current_pulse_width;

    public:

      ConfiguredServo(uint8_t pin_number);
      ConfiguredServo(uint8_t pin_number, uint16_t minimum_pulse_width, uint16_t maximum_pulse_width,
                      uint16_t center_pulse_width, bool enforce);
      ~ConfiguredServo();
      void setup();
      void setup(const Configuration & new_configuration);
      void setPosition(int16_t position);
      void setPulseWidth(uint16_t pulse_width);
      void setDefaultConfiguration();
      void setConfiguration(const Configuration & new_configuration);
      Configuration getConfiguration() const;

    private:

      uint16_t correctPulseWidth(uint16_t requested_pulse_width) const;

  };

}


#endif