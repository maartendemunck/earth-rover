#ifndef __NEOGPS_WRAPPER__
#define __NEOGPS_WRAPPER__


#include <cstdint>
#include <GPSport.h>
#include <NMEAGPS.h>


namespace earth_rover
{

  template <typename SerialDevice, int8_t serial_device_rx_pin = -1, int8_t serial_device_tx_pin = -1>
  class NeoGpsWrapper
  {
    private:

      SerialDevice & serial_device;
      NMEAGPS gps_device;
      gps_fix current_gps_fix;

    public:

      NeoGpsWrapper(SerialDevice & serial_device):
        serial_device {serial_device}
      {
        ;
      }

      ~NeoGpsWrapper() = default;

      void setup()
      {
        // Initialise serial device.
        if(serial_device_rx_pin >= 0)
        {
          serial_device.setRX(serial_device_rx_pin);
        }
        if(serial_device_tx_pin >= 0)
        {
          serial_device.setTX(serial_device_tx_pin);
        }
        serial_device.begin(9600, SERIAL_8N1);
      }

      inline void spinOnce()
      {
        // Process available data and update gps fix.
        while(gps_device.available(serial_device))
        {
          current_gps_fix = gps_device.read();
        }
      }

      inline gps_fix getCurrentGpsFix()
      {
        return current_gps_fix;
      }

  };

}

#endif