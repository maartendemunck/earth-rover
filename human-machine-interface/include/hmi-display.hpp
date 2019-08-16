#ifndef __HMI_DISPLAY__
#define __HMI_DISPLAY__


#include <car-configuration.hpp>


namespace earth_rover
{

  template <typename SerialDevice>
  class HmiDisplay
  {
    private:

      SerialDevice & serial_device;
      static constexpr uint16_t rx_buffer_size{8};
      uint8_t rx_buffer[rx_buffer_size];
      uint16_t rx_buffer_pointer;

      CarConfiguration & car_configuration;


    public:

      HmiDisplay(SerialDevice & serial_device, CarConfiguration & car_configuration);
      ~HmiDisplay();

      void setup();
      void receiveData();

    private:

      void updateSettingsOnDisplay (bool force_update = false);



  };

  template <typename SerialDevice>
  HmiDisplay<SerialDevice>::HmiDisplay(SerialDevice & serial_device, CarConfiguration & car_configuration):
    serial_device {serial_device},
    rx_buffer_pointer {0u},
    car_configuration {car_configuration}
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
    // Switch to page 0 (speedometer).
    serial_device.print("page 0\xff\xff\xff");
    serial_device.flush();
    // Send current settings to display.
    updateSettingsOnDisplay(true);
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
  void HmiDisplay<SerialDevice>::updateSettingsOnDisplay (bool force_update)
  {
    // Update steering servo settings.
    if(force_update || car_configuration.getSteeringServoConfiguration().isConfigurationChanged())
    {
      auto steering_servo_config = car_configuration.getSteeringServoConfiguration().resetConfigurationChanged();
      serial_device.printf("set_steering.var_left.val=%d\xff\xff\xff", steering_servo_config.minimum);
      serial_device.printf("set_steering.var_center.val=%d\xff\xff\xff", steering_servo_config.center);
      serial_device.printf("set_steering.var_right.val=%d\xff\xff\xff", steering_servo_config.maximum);
      serial_device.printf("set_steering.var_channel.val=%d\xff\xff\xff", steering_servo_config.input_channel);
      serial_device.flush();
      // TODO: if the display shows the steering servo configuration, reload the page.
    }
    // Update speed controller settings.
    if(force_update || car_configuration.getSpeedControllerConfiguration().isConfigurationChanged())
    {
      auto speed_controller_config = car_configuration.getSpeedControllerConfiguration().resetConfigurationChanged();
      serial_device.printf("set_throttle.var_reverse.val=%d\xff\xff\xff", speed_controller_config.minimum);
      serial_device.printf("set_throttle.var_stop.val=%d\xff\xff\xff", speed_controller_config.center);
      serial_device.printf("set_throttle.var_forward.val=%d\xff\xff\xff", speed_controller_config.maximum);
      serial_device.printf("set_throttle.var_channel.val=%d\xff\xff\xff", speed_controller_config.input_channel);
      serial_device.flush();
      // TODO: if the display shows the throttle configuration, reload the page.
    }
    // Update gearbox servo settings.
    if(force_update || car_configuration.getGearboxServoConfiguration().isConfigurationChanged())
    {
      auto gearbox_servo_config = car_configuration.getGearboxServoConfiguration().resetConfigurationChanged();
      serial_device.printf("set_gearbox.var_low.val=%d\xff\xff\xff", gearbox_servo_config.minimum);
      serial_device.printf("set_gearbox.var_neutral.val=%d\xff\xff\xff", gearbox_servo_config.center);
      serial_device.printf("set_gearbox.var_high.val=%d\xff\xff\xff", gearbox_servo_config.maximum);
      serial_device.printf("set_gearbox.var_channel.val=%d\xff\xff\xff", gearbox_servo_config.input_channel);
      serial_device.flush();
      // TODO: if the display shows the gearbox configuration, reload the page.
    }
    // Update the radio settings.
    if(force_update || car_configuration.getRadioConfiguration().isConfigurationChanged())
    {
      auto radio_config = car_configuration.getRadioConfiguration().resetConfigurationChanged();
      serial_device.printf("set_radio.var_tx_power.val=%d\xff\xff\xff", radio_config.tx_power);
      serial_device.printf("set_radio.var_rx_power.val=%d\xff\xff\xff", radio_config.rx_power);
      serial_device.printf("set_radio.var_channel.val=%d\xff\xff\xff", radio_config.channel);
      serial_device.flush();
      // TODO: if the display shows the radio configuration, reload the page.
    }
  }

}

#endif