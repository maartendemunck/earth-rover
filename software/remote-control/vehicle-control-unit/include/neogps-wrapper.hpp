//! GPS device driver for the Earth Rover's VCU (interface and template implementation).
/*!
 *  This GPS device driver encapsulates a Teensy's serial device and a NeoGPS instance.
 *
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */

#ifndef __EARTH_ROVER_VCU__NEOGPS_WRAPPER__
#define __EARTH_ROVER_VCU__NEOGPS_WRAPPER__

#include <GPSport.h>
#include <NMEAGPS.h>
#include <cstdint>

namespace earth_rover_vcu {

    //! GPS device driver for the Earth Rover's VCU.
    /*!
     *  This GPS device driver encapsulates a Teensy's serial device and a NeoGPS NMEA sentence
     * interpreter.
     *
     *  \tparam SerialDevice Type of the serial device connected to the GPS device.
     *
     *  \ingroup VCU
     */
    template <typename SerialDevice> class NeoGpsWrapper {
      private:
        SerialDevice &serial_device;  //!< Serial device used by the GPS receiver.
        int8_t rx_pin;                //!< Rx pin used by the serial device.
        int8_t tx_pin;                //!< Tx pin used by the serial device.
        NMEAGPS gps_device;           //!< NeoGPS NMEA sentence interpreter.
        gps_fix current_gps_fix;      //!< Most recent GPS fix.

      public:
        //! Constructor.
        /*!
         *  \param serial_device Serial device.
         *  \param rx_pin Rx pin used by the serial device (<0 for default rx pin).
         *  \param tx_pin Tx pin used by the serial device (<0 for default tx pin).
         */
        NeoGpsWrapper(SerialDevice &serial_device, int8_t rx_pin = -1, int8_t tx_pin = -1)
            : serial_device{serial_device}, rx_pin{rx_pin}, tx_pin{tx_pin}, gps_device{} {
            ;
        }

        //! Default destructor.
        ~NeoGpsWrapper() = default;

        //! Initialize the GPS device.
        void setup() {
            // Initialise serial device.
            if(rx_pin >= 0)  // Negative number: use default pin.
            {
                serial_device.setRX(rx_pin);
            }
            if(tx_pin >= 0)  // Negative number: use default pin.
            {
                serial_device.setTX(tx_pin);
            }
            serial_device.begin(9600, SERIAL_8N1);
        }

        //! Spinning loop.
        /*!
         *  Read and process received GPS data.
         */
        inline void spinOnce() {
            // Process available data and update gps fix.
            while(gps_device.available(serial_device)) {
                current_gps_fix = gps_device.read();
            }
        }

        //! Get the most recent GPS fix.
        /*!
         *  \return The most recent GPS fix in NeoGPS gps_fix format.
         */
        inline gps_fix getCurrentGpsFix() { return current_gps_fix; }
    };

}  // namespace earth_rover_vcu

#endif