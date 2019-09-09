#ifndef __HMI_DISPLAY__
#define __HMI_DISPLAY__


#include "car-configuration.hpp"
#include "car-state.hpp"
#include "from-to-integral.hpp"
#include "limit-value.hpp"


namespace earth_rover
{

  template <typename SerialDevice>
  class HmiDisplay
  {
    private:

      enum class HmiPage: uint8_t
      {
        Speedometer = 0x01u,
        Orientation = 0x02u,
        Location = 0x03u,
        SteeringSettings = 0x11u,
        ThrottleSettings = 0x12u,
        GearboxSettings = 0x13u,
        RadioSettings = 0x14u
      };

      enum class Light: uint8_t
      {
        TurnSignalRight = 0x01u,
        TurnSignalLeft = 0x02u,
        DippedBeam = 0x03u,
        HighBeam = 0x04u,
        HazardFlashers = 0x05u
      };

      SerialDevice & serial_device;
      static constexpr uint16_t rx_buffer_size{8};
      uint8_t rx_buffer[rx_buffer_size];
      uint16_t rx_buffer_pointer;

      CarConfiguration & car_configuration;
      CarState & car_state;

      HmiPage current_page;

    public:

      HmiDisplay(SerialDevice & serial_device, CarConfiguration & car_configuration, CarState & car_state);
      ~HmiDisplay();

      void setup();
      void spinOnce();

      void publishVelocity(float velocity);
      void publishOdometer(float odometer);
      void publishTripmeter(float tripmeter);

      void receiveData();

    private:

      void setTurnSignalRight(bool state);
      void setTurnSignalLeft(bool state);
      void updateSettingsOnDisplay(bool force_update = false);

  };


  template <typename SerialDevice>
  HmiDisplay<SerialDevice>::HmiDisplay(SerialDevice & serial_device,
                                       CarConfiguration & car_configuration, CarState & car_state):
    serial_device {serial_device},
    rx_buffer_pointer {0u},
    car_configuration {car_configuration},
    car_state {car_state}
  {
    ;
  }


  template <typename SerialDevice>
  HmiDisplay<SerialDevice>::~HmiDisplay()
  {
    // Restore default baudrate.
    serial_device.printf("baud 9600\xff\xff\xff");
    serial_device.flush();
    serial_device.end();
  }


  template <typename SerialDevice>
  void HmiDisplay<SerialDevice>::setup()
  {
    // If the Nextion display is configured for 9600 baud (the default
    // setting), switch to 115200 baud. If the display is already configured
    // for 115200 baud, this code will have no effect.
    serial_device.begin(9600u);
    delay(20);
    serial_device.print("bkcmd=0\xff\xff\xff");
    serial_device.print("baud=115200\xff\xff\xff");
    serial_device.flush();
    delay(60);
    serial_device.begin(115200u);
    delay(20);
    serial_device.print("\xff\xff\xff");
    serial_device.print("bkcmd=0\xff\xff\xff");
    // Send current settings to display.
    updateSettingsOnDisplay(true);
    // Switch to speedometer page.
    serial_device.print("page speedometer\xff\xff\xff");
    serial_device.flush();
    current_page = HmiPage::Speedometer;
  }


  template <typename SerialDevice>
  void HmiDisplay<SerialDevice>::spinOnce()
  {
    if(car_state.getTurnSignalRightCancelled(true))
    {
      setTurnSignalRight(false);
    }
    if(car_state.getTurnSignalLeftCancelled(true))
    {
      setTurnSignalLeft(false);
    }
    updateSettingsOnDisplay();
  }


  template <typename SerialDevice>
  void HmiDisplay<SerialDevice>::publishVelocity(float velocity)
  {
    uint16_t angle = (320u + uint16_t(26 * limit_value(velocity, 0., 10.))) % 360;
    serial_device.printf("speedometer.z_speedometer.val=%u\xff\xff\xff", angle);
  }


  template <typename SerialDevice>
  void HmiDisplay<SerialDevice>::publishOdometer(float odometer)
  {
    int32_t odometer_raw = int32_t(odometer * 100) % 1000000;
    serial_device.printf("speedometer.x_odo.val=%d\xff\xff\xff", odometer_raw);
  }


  template <typename SerialDevice>
  void HmiDisplay<SerialDevice>::publishTripmeter(float tripmeter)
  {
    int32_t tripmeter_raw = int32_t(tripmeter * 1000) % 1000000;
    serial_device.printf("speedometer.x_trip.val=%d\xff\xff\xff", tripmeter_raw);
  }


  template <typename SerialDevice>
  void HmiDisplay<SerialDevice>::receiveData()
  {
    while(serial_device.available())
    {
      rx_buffer[rx_buffer_pointer ++] = char(serial_device.read() & 0x00ffu);
      if(rx_buffer_pointer >= 3
          && rx_buffer[rx_buffer_pointer - 1] == 0xffu
          && rx_buffer[rx_buffer_pointer - 2] == 0xffu
          && rx_buffer[rx_buffer_pointer - 3] == 0xffu)
      {
        // Print response.
        {
          Serial.print("Data from Nextion display:");
          for(auto i = 0; i < rx_buffer_pointer - 3; ++ i)
          {
            Serial.printf(" %02x", rx_buffer[i]);
          }
          Serial.println("");
        }
        // Process response.
        if(rx_buffer[0] == 0xa0u && rx_buffer_pointer == 5)  // Page change.
        {
          current_page = from_integral<HmiPage>(rx_buffer[1]);
        }
        else if(rx_buffer[0] == 0xa1u && rx_buffer_pointer == 6)  // Lighting.
        {
          Light light = from_integral<Light>(rx_buffer[1]);
          bool state = rx_buffer[2];
          switch(light)
          {
            case Light::TurnSignalRight:
              car_state.setTurnSignalRight(state);
              Serial.printf("Turn signal right: %d\n", int16_t(state));
              break;
            case Light::TurnSignalLeft:
              car_state.setTurnSignalLeft(state);
              Serial.printf("Turn signal left: %d\n", int16_t(state));
              break;
            case Light::DippedBeam:
              car_state.setDippedBeam(state);
              break;
            case Light::HighBeam:
              car_state.setHighBeam(state);
              break;
            case Light::HazardFlashers:
              car_state.setHazardFlashers(state);
              break;
          }
        }
        else if(rx_buffer[0] == 0xa2u && rx_buffer_pointer == 8)  // Settings.
        {
          HmiPage page = from_integral<HmiPage>(rx_buffer[1]);
          uint8_t setting = rx_buffer[2];
          uint16_t value = (rx_buffer[4] << 8) + rx_buffer[3];
          Serial.printf("Setting %02x:%02x changed to %u\n", to_integral(page), setting, value);
          switch(page)
          {
            case HmiPage::Speedometer:
              // No settings on speedometer page.
              break;
            case HmiPage::Orientation:
              // No settings on orientation page.
              break;
            case HmiPage::Location:
              // No setting on location page.
              break;
            case HmiPage::SteeringSettings:
              switch(setting)
              {
                case 0:
                  car_configuration.getSteeringConfig().setInputChannel(value, Configuration::Changed::Display);
                  break;
                case 1:
                  car_configuration.getSteeringConfig().setMinimum(value, Configuration::Changed::Display);
                  break;
                case 2:
                  car_configuration.getSteeringConfig().setCenter(value, Configuration::Changed::Display);
                  break;
                case 3:
                  car_configuration.getSteeringConfig().setMaximum(value, Configuration::Changed::Display);
                  break;
              }
              break;
            case HmiPage::ThrottleSettings:
              switch(setting)
              {
                case 0:
                  car_configuration.getThrottleConfig().setInputChannel(value, Configuration::Changed::Display);
                  break;
                case 1:
                  car_configuration.getThrottleConfig().setMinimum(value, Configuration::Changed::Display);
                  break;
                case 2:
                  car_configuration.getThrottleConfig().setCenter(value, Configuration::Changed::Display);
                  break;
                case 3:
                  car_configuration.getThrottleConfig().setMaximum(value, Configuration::Changed::Display);
                  break;
              }
              break;
            case HmiPage::GearboxSettings:
              switch(setting)
              {
                case 0:
                  car_configuration.getGearboxConfig().setInputChannel(value, Configuration::Changed::Display);
                  break;
                case 1:
                  car_configuration.getGearboxConfig().setCenter(value, Configuration::Changed::Display);
                  break;
                case 2:
                  car_configuration.getGearboxConfig().setMinimum(value, Configuration::Changed::Display);
                  break;
                case 3:
                  car_configuration.getGearboxConfig().setMaximum(value, Configuration::Changed::Display);
                  break;
              }
              break;
            case HmiPage::RadioSettings:
              switch(setting)
              {
                case 1:
                  car_configuration.getRadioConfig().setTxPower(value, Configuration::Changed::Display);
                  break;
                case 2:
                  car_configuration.getRadioConfig().setRxPower(value, Configuration::Changed::Display);
                  break;
              }
              break;
          }
        }
        // Reset receive buffer pointer.
        rx_buffer_pointer = 0;
      }
      // Prevent buffer overflows.
      if(rx_buffer_pointer >= rx_buffer_size)
      {
        rx_buffer_pointer = rx_buffer_size - 1;
        for(uint16_t i = 1; i < rx_buffer_size; ++ i)
        {
          rx_buffer[i - 1] = rx_buffer[i];
        }
        Serial.println("Read buffer for Nextion data overflowed");
      }
    }
  }


  template <typename SerialDevice>
  void HmiDisplay<SerialDevice>::setTurnSignalRight(bool state)
  {
    serial_device.printf("speedometer.var_tsright.val=%d\xff\xff\xff", int16_t(state));
  }


  template <typename SerialDevice>
  void HmiDisplay<SerialDevice>::setTurnSignalLeft(bool state)
  {
    serial_device.printf("speedometer.var_tsleft.val=%d\xff\xff\xff", int16_t(state));
  }


  template <typename SerialDevice>
  void HmiDisplay<SerialDevice>::updateSettingsOnDisplay (bool force_update)
  {
    // Update steering servo settings.
    if(force_update || car_configuration.getSteeringConfig().isConfigurationChanged())
    {
      auto steering_servo_config = car_configuration.getSteeringConfig().resetConfigurationChanged();
      serial_device.printf("set_steering.var_left.val=%d\xff\xff\xff", steering_servo_config.minimum);
      serial_device.printf("set_steering.var_center.val=%d\xff\xff\xff", steering_servo_config.center);
      serial_device.printf("set_steering.var_right.val=%d\xff\xff\xff", steering_servo_config.maximum);
      serial_device.printf("set_steering.var_channel.val=%d\xff\xff\xff", steering_servo_config.input_channel);
      serial_device.flush();
      if(current_page == HmiPage::SteeringSettings)
      {
        serial_device.print("page set_steering\xff\xff\xff");
      }
    }
    // Update electronic speed controller settings.
    if(force_update || car_configuration.getThrottleConfig().isConfigurationChanged())
    {
      auto speed_controller_config = car_configuration.getThrottleConfig().resetConfigurationChanged();
      serial_device.printf("set_throttle.var_reverse.val=%d\xff\xff\xff", speed_controller_config.minimum);
      serial_device.printf("set_throttle.var_stop.val=%d\xff\xff\xff", speed_controller_config.center);
      serial_device.printf("set_throttle.var_forward.val=%d\xff\xff\xff", speed_controller_config.maximum);
      serial_device.printf("set_throttle.var_channel.val=%d\xff\xff\xff", speed_controller_config.input_channel);
      serial_device.flush();
      if(current_page == HmiPage::ThrottleSettings)
      {
        serial_device.print("page set_throttle\xff\xff\xff");
      }
    }
    // Update gearbox servo settings.
    if(force_update || car_configuration.getGearboxConfig().isConfigurationChanged())
    {
      auto gearbox_servo_config = car_configuration.getGearboxConfig().resetConfigurationChanged();
      serial_device.printf("set_gearbox.var_low.val=%d\xff\xff\xff", gearbox_servo_config.minimum);
      serial_device.printf("set_gearbox.var_neutral.val=%d\xff\xff\xff", gearbox_servo_config.center);
      serial_device.printf("set_gearbox.var_high.val=%d\xff\xff\xff", gearbox_servo_config.maximum);
      serial_device.printf("set_gearbox.var_channel.val=%d\xff\xff\xff", gearbox_servo_config.input_channel);
      serial_device.flush();
      if(current_page == HmiPage::GearboxSettings)
      {
        serial_device.print("page set_gearbox\xff\xff\xff");
      }
    }
    // Update the radio settings.
    if(force_update || car_configuration.getRadioConfig().isConfigurationChanged())
    {
      auto radio_config = car_configuration.getRadioConfig().resetConfigurationChanged();
      serial_device.printf("set_radio.var_tx_power.val=%d\xff\xff\xff", radio_config.tx_power);
      serial_device.printf("set_radio.var_rx_power.val=%d\xff\xff\xff", radio_config.rx_power);
      serial_device.flush();
      if(current_page == HmiPage::RadioSettings)
      {
        serial_device.print("page set_radio\xff\xff\xff");
      }
    }
  }

}

#endif