// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__NEOGPS_WRAPPER__
#define __EARTH_ROVER__NEOGPS_WRAPPER__

#include <GPSport.h>
#include <NMEAGPS.h>
#include <cstdint>

namespace earth_rover {

    template <typename SerialDevice> class NeoGpsWrapper {
      private:
        SerialDevice &serial_device;
        int8_t rx_pin;
        int8_t tx_pin;
        NMEAGPS gps_device;
        gps_fix current_gps_fix;

      public:
        NeoGpsWrapper(SerialDevice &serial_device, int8_t rx_pin = -1, int8_t tx_pin = -1)
            : serial_device{serial_device}, rx_pin{rx_pin}, tx_pin{tx_pin}, gps_device{} {
            ;
        }

        NeoGpsWrapper(NeoGpsWrapper &&original)
            : serial_device{original.serial_device}, rx_pin{original.rx_pin},
              tx_pin{original.tx_pin}, gps_device{}, current_gps_fix{
                                                         std::move(original.current_gps_fix)} {
            ;
        }

        void setup() {
            // Initialise serial device.
            if(rx_pin >= 0) {
                serial_device.setRX(rx_pin);
            }
            if(tx_pin >= 0) {
                serial_device.setTX(tx_pin);
            }
            serial_device.begin(9600, SERIAL_8N1);
        }

        inline void spinOnce() {
            while(gps_device.available(serial_device)) {
                current_gps_fix = gps_device.read();
            }
        }

        inline gps_fix getCurrentGpsFix() {
            return current_gps_fix;
        }
    };

}  // namespace earth_rover

#endif