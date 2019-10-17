//! Nextion HMI touch display device driver for the Earth Rover (interface and template implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_HMI__NEXTION_HMI_DISPLAY__
#define __EARTH_ROVER_HMI__NEXTION_HMI_DISPLAY__


#include "car-state.hpp"
#include "from-to-integral.hpp"
#include "limit-value.hpp"


namespace earth_rover_hmi
{

  //! Nextion HMI touch display device driver for the Earth Rover.
  /*!
   *  \tparam SerialDevice_t Type of the serial device connected to the Nextion HMI touch display.
   * 
   *  \ingroup HMI
   */
  template <typename SerialDevice_t>
  class NextionHmiDisplay
  {
    private:

      //! Information and configuration pages.
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

      //! Automotive lighting IDs.
      enum class Light: uint8_t
      {
        TurnSignalRight = 0x01u,
        TurnSignalLeft = 0x02u,
        DippedBeam = 0x03u,
        HighBeam = 0x04u,
        HazardFlashers = 0x05u
      };

      SerialDevice_t & serial_device;                 //!< Serial device used to communicate with the Nextion display.
      static constexpr uint16_t rx_buffer_size{8};  //!< Size of the receive buffer (can contain the largest message).
      uint8_t rx_buffer[rx_buffer_size];            //!< Receive buffer.
      uint16_t rx_buffer_pointer;                   //!< Relative location of the next byte in the receive buffer.

      CarState & car_state;                         //!< Current car state (digital twin).

      HmiPage current_page;                         //!< Active information or configuration page.
      bool configuration_pages_available;           //!< True if the configuration pages are available, false if not.

    public:

      //! Constructor.
      /*!
       *  \param serial_device Serial device used to communicate with the Nextion HMI touch display.
       *  \param car_state Car configuration and state (digital twin).
       */
      NextionHmiDisplay(SerialDevice_t & serial_device, CarState & car_state)
      :
        serial_device {serial_device},
        rx_buffer_pointer {0u},
        car_state {car_state},
        configuration_pages_available {false}
      {
        ;
      }

      //! Destructor.
      /*!
       *  The destructor restores the default baud rate.
       */
      ~NextionHmiDisplay()
      {
        // Restore default baudrate.
        serial_device.printf("baud 9600\xff\xff\xff");
        serial_device.flush();
        serial_device.end();
      }

      //! Initialize the Nextion HMI touch display device.
      /*!
       *  Switch the display to 115200 baud, disable command responses and switch to the speedometer page.
       */
      void setup()
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
        // Disable configuration pages.
        disableConfigurationPages();
        // Send current settings to display.
        updateSettingsOnDisplay(true);
        // Switch to speedometer page.
        serial_device.print("page speedometer\xff\xff\xff");
        serial_device.flush();
        current_page = HmiPage::Speedometer;
      }

      //! Spinning loop.
      /*!
       *  The spinning loop updates the display (cancelled turn signals and new sensor measurements).
       */
      void spinOnce()
      {
        if(!configuration_pages_available && car_state.isConfigurationAvailable())
        {
          enableConfigurationPages();
        }
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

      //! Callback to process data received from the serial port.
      void receiveData()
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
              auto new_page = from_integral<HmiPage>(rx_buffer[1]);
              if(new_page != current_page)
              {
                current_page = new_page;
                updateSettingsOnDisplay(true);
              }
            }
            else if(rx_buffer[0] == 0xa1u && rx_buffer_pointer == 6)  // Lighting.
            {
              Light light = from_integral<Light>(rx_buffer[1]);
              bool state = rx_buffer[2];
              switch(light)
              {
                case Light::TurnSignalRight:
                  car_state.setTurnSignalRight(state);
                  break;
                case Light::TurnSignalLeft:
                  car_state.setTurnSignalLeft(state);
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
                      car_state.setSteeringInputChannel(value - 1);
                      break;
                    case 1:
                      car_state.setSteerLeftPulseWidth(value);
                      break;
                    case 2:
                      car_state.setSteerCenterPulseWidth(value);
                      break;
                    case 3:
                      car_state.setSteerRightPulseWidth(value);
                      break;
                  }
                  break;
                case HmiPage::ThrottleSettings:
                  switch(setting)
                  {
                    case 0:
                      car_state.setThrottleInputChannel(value - 1);
                      break;
                    case 1:
                      car_state.setFullBackwardsPulseWidth(value);
                      break;
                    case 2:
                      car_state.setStopPulseWidth(value);
                      break;
                    case 3:
                      car_state.setFullForwardPulseWidth(value);
                      break;
                  }
                  break;
                case HmiPage::GearboxSettings:
                  switch(setting)
                  {
                    case 0:
                      car_state.setGearboxInputChannel(value - 1);
                      break;
                    case 1:
                      car_state.setGearPulseWidth(0, value);
                      break;
                    case 2:
                      car_state.setGearPulseWidth(1, value);
                      break;
                    case 3:
                      car_state.setGearPulseWidth(2, value);
                      break;
                  }
                  break;
                case HmiPage::RadioSettings:
                  switch(setting)
                  {
                    case 1:
                      car_state.setHmiRadioPower(value);
                      break;
                    case 2:
                      car_state.setVcuRadioPower(value);
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

    private:

      //! Enable the configuration pages.
      void enableConfigurationPages()
      {
        serial_device.print("speedometer.var_settings.val=1\xff\xff\xff");
        serial_device.flush();
        configuration_pages_available = true;
      }

      //! Enable the configuration pages.
      void disableConfigurationPages()
      {
        serial_device.print("speedometer.var_settings.val=0\xff\xff\xff");
        serial_device.flush();
        configuration_pages_available = false;
      }

      //! Update the right turn signal's button on the display.
      /*!
       *  \param state New state of the right turn signal's button.
       */
      void setTurnSignalRight(bool state)
      {
        serial_device.printf("speedometer.var_tsright.val=%d\xff\xff\xff", int16_t(state));
        serial_device.flush();
      }

      //! Update the left turn signal's button on the display.
      /*!
       *  \param state New state of the left turn signal's button.
       */
      void setTurnSignalLeft(bool state)
      {
        serial_device.printf("speedometer.var_tsleft.val=%d\xff\xff\xff", int16_t(state));
        serial_device.flush();
      }

      //! Update displayed measurements and configuration parameters.
      /*!
       *  \param force_update If true, update all visualizations, if false, update changed visualizations.
       */
      void updateSettingsOnDisplay(bool force_update = false)
      {
        // Update speedometer.
        if(current_page == HmiPage::Speedometer && (force_update || car_state.isSpeedometerUpdated()))
        {
          auto speedometer = car_state.getSpeedometer();
          if(speedometer.valid == true)
          {
            uint16_t angle = (320u + uint16_t(26 * limit_value(speedometer.data.speed, 0., 10.))) % 360u;
            serial_device.printf("z_speed.val=%d\xff\xff\xff", angle);
            uint32_t odometer_raw = int32_t(speedometer.data.odometer * 100) % 1000000;
            serial_device.printf("x_odo.val=%d\xff\xff\xff", odometer_raw);
            int32_t tripmeter_raw = int32_t(speedometer.data.tripmeter * 1000) % 1000000;
            serial_device.printf("x_trip.val=%d\xff\xff\xff", tripmeter_raw);
          }
          else
          {
            serial_device.print("z_speed.val=320\xff\xff\xff");
          }
          serial_device.flush();
        }
        // Update orientation.
        if(current_page == HmiPage::Orientation && (force_update || car_state.isOrientationUpdated()))
        {
          auto orientation = car_state.getOrientation();
          if(orientation.valid == true)
          {
            serial_device.printf("z_yaw.val=%d\xff\xff\xff", (90 + int16_t(orientation.data.yaw)) % 360);
            serial_device.printf("z_pitch.val=%d\xff\xff\xff",
                (90 + 2 * int16_t(limit_value(orientation.data.pitch, -60., 60.))) % 360);
            serial_device.printf("z_roll.val=%d\xff\xff\xff",
                (90 + 2 * int16_t(limit_value(orientation.data.roll, -60., 60.))) % 360);
            serial_device.flush();
          }
          else
          {
            ;  // TODO: Remove pointer? Don't forget to remove serial_device.flush() below if block.
          }
        }
        // Update location.
        if(current_page == HmiPage::Location && (force_update || car_state.isLocationUpdated()))
        {
          auto location = car_state.getLocation();
          if(location.valid == true)
          {
            serial_device.printf("t_lat.txt=\"%d\xb0%02d'%02d.%02d\\\" %c\"\xff\xff\xff",
                                location.data.latitude.degrees, location.data.latitude.minutes,
                                location.data.latitude.seconds_whole, location.data.latitude.seconds_frac / 100,
                                location.data.latitude.hemisphere == Hemisphere_t::NORTH_H? 'N': 'S');
            serial_device.printf("t_lon.txt=\"%d\xb0%02d'%02d.%02d\\\" %c\"\xff\xff\xff",
                                location.data.longitude.degrees, location.data.longitude.minutes,
                                location.data.longitude.seconds_whole, location.data.longitude.seconds_frac / 100,
                                location.data.longitude.hemisphere == Hemisphere_t::EAST_H? 'E': 'W');
          }
          else
          {
            serial_device.printf("t_lat.txt=\"NO GPS FIX\"\xff\xff\xff");
            serial_device.printf("t_lon.txt=\"NO GPS FIX\"\xff\xff\xff");
          }
          serial_device.flush();
        }
        // Update altitude.
        if(current_page == HmiPage::Location && (force_update || car_state.isAltitudeUpdated()))
        {
          auto altitude = car_state.getAltitude();
          if(altitude.valid == true)
          {
            serial_device.printf("t_alt.txt=\"%d.%02dm\"\xff\xff\xff", altitude.data / 100, abs(altitude.data) % 100);
          }
          else
          {
            serial_device.printf("t_alt.txt=\"NO GPS FIX\"\xff\xff\xff");
          }
          
          serial_device.flush();
        }
        // Update steering settings.
        if(current_page == HmiPage::SteeringSettings && force_update)
        {
          auto config = car_state.getSteeringConfiguration();
          serial_device.printf("set_steering.var_left.val=%u\xff\xff\xff", config.pulse_width_minimum);
          serial_device.printf("set_steering.var_center.val=%u\xff\xff\xff", config.pulse_width_center);
          serial_device.printf("set_steering.var_right.val=%u\xff\xff\xff", config.pulse_width_maximum);
          serial_device.printf("set_steering.var_channel.val=%u\xff\xff\xff", config.input_channel + 1);
          serial_device.print("page set_steering\xff\xff\xff");
          serial_device.flush();
        }
        if(current_page == HmiPage::ThrottleSettings && force_update)
        {
          auto config = car_state.getThrottleConfiguration();
          serial_device.printf("set_throttle.var_reverse.val=%u\xff\xff\xff", config.pulse_width_minimum);
          serial_device.printf("set_throttle.var_stop.val=%u\xff\xff\xff", config.pulse_width_center);
          serial_device.printf("set_throttle.var_forward.val=%u\xff\xff\xff", config.pulse_width_maximum);
          serial_device.printf("set_throttle.var_channel.val=%u\xff\xff\xff", config.input_channel + 1);
          serial_device.print("page set_throttle\xff\xff\xff");
          serial_device.flush();
        }
        if(current_page == HmiPage::GearboxSettings && force_update)
        {
          auto config = car_state.getGearboxConfiguration();
          serial_device.printf("set_gearbox.var_neutral.val=%d\xff\xff\xff", config.getPulseWidth(0));
          serial_device.printf("set_gearbox.var_low.val=%d\xff\xff\xff", config.getPulseWidth(1));
          serial_device.printf("set_gearbox.var_high.val=%d\xff\xff\xff", config.getPulseWidth(2));
          serial_device.printf("set_gearbox.var_channel.val=%d\xff\xff\xff", config.input_channel + 1);
          serial_device.print("page set_gearbox\xff\xff\xff");
          serial_device.flush();
        }
      }

/*
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
*/

  };

}

#endif