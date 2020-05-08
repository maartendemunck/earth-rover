//! Car state (digital twin) for the Earth Rover (interface and inline part of the implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */

#ifndef __EARTH_ROVER_HMI__CAR_STATE__
#define __EARTH_ROVER_HMI__CAR_STATE__

#include "gearbox-servo-state.hpp"
#include "limit-value.hpp"
#include "radio-state.hpp"
#include "sensor-state.hpp"
#include "servo-state.hpp"
#include <DMS.h>
#include <cstdint>

namespace {
    //! Compare two DMS_t objects.
    /*!
     *  \param lhs Left hand side operand.
     *  \param rhs Right hand size operand.
     *  \return True if both objects are different, false if they are equal.
     *
     *  \ingroup HMI
     */
    bool operator!=(const DMS_t &lhs, const DMS_t &rhs) {
        return (lhs.degrees != rhs.degrees || lhs.minutes != rhs.minutes
                || lhs.hemisphere != rhs.hemisphere || lhs.seconds_whole != rhs.seconds_whole
                || lhs.seconds_frac != rhs.seconds_frac);
    }
}  // namespace

namespace earth_rover_hmi {

    //! Earth Rover car state (digital twin).
    /*!
     *  This class models the current state of the Earth Rover.
     *
     *  \ingroup HMI
     */
    class CarState {
      public:
        //! Automotive lighting state.
        /*!
         *  \ingroup HMI
         */
        struct Lighting {
            bool turn_signal_right;  //!< Right turn signal.
            bool turn_signal_left;   //!< Left turn signal.
            bool dipped_beam;        //!< Dipped beam headlamps.
            bool high_beam;          //!< High beam headlamps.
            bool hazard_flashers;    //!< Hazard flashers.
        };

        //! Speedometer measurement.
        /*
         *  \ingroup HMI
         */
        struct Speedometer {
            float speed;      //!< Speed (in km/h).
            float odometer;   //!< Odometer (in km).
            float tripmeter;  //!< Trip meter (in km).
            //! Overloaded comparison operator.
            /*!
             *  \param rhs Right hand side operand.
             *  \return true if both operands are different, false if they are equal.
             */
            bool operator!=(const Speedometer &rhs) {
                return (speed != rhs.speed || odometer != rhs.odometer
                        || tripmeter != rhs.tripmeter);
            }
        };

        //! IMU measurement.
        /*!
         *  \ingroup HMI
         */
        struct Orientation {
            float yaw;    //!< Yaw (rotation around the vertical axis) (in degrees).
            float pitch;  //!< Pitch (rotation around the transverse axis) (in degrees).
            float roll;   //!< Roll (rotation around the longitudinal axis) (in degrees).
            //! Overloaded comparison operator.
            /*!
             *  \param rhs Right hand side operand.
             *  \return true if both operands are different, false if they are equal.
             */
            bool operator!=(const Orientation &rhs) {
                return (yaw != rhs.yaw || pitch != rhs.pitch || roll != rhs.roll);
            }
        };

        //! GPS location measurement.
        /*!
         *  \ingroup HMI
         */
        struct Location {
            DMS_t latitude;   //!< Latitude.
            DMS_t longitude;  //!< Longitude.
            //! Overloaded comparison operator.
            /*!
             *  \param rhs Right hand side operand.
             *  \return true if both operands are different, false if they are equal.
             */
            bool operator!=(const Location &rhs) {
                return (latitude != rhs.latitude || longitude != rhs.longitude);
            }
        };

      private:
        //! Flag indicating whether the configuration stored in the VCU is available.
        bool configuration_available;
        //! Flag indicating that the current configuration should be saved to non-volatile memory.
        bool configuration_save_request;

        //! Steering servo configuration and state.
        ServoState steering_servo;
        //! ESC configuration and state.
        ServoState esc;
        //! Gearbox servo configuration.
        GearboxServoState<0, 1, 2> gearbox_servo;  //! TODO replace hardcoded gearbox configuration.
        //! Flag used to implement the hysteresis in the shift down and up commands.
        int8_t gearbox_locked = 0;

        //! Current state of the automotive lighting.
        Lighting lighting;
        //! Flag used to implement the hysteresis to automatically shut off the turn signals.
        int8_t turn_signal_free{0};
        //! Flag used to shut off the left turn signal using the steering command.
        bool turn_signal_left_cancelled{false};
        //! Flag used to shut off the right turn signal using the steering command.
        bool turn_signal_right_cancelled{false};

        //! Radio state.
        RadioState radio;

        //! Speed, odometer and trip meter measurement.
        SensorState<Speedometer> speedometer;
        //! Orientation measurement.
        SensorState<Orientation> orientation;
        //! GPS location (latitude and longitude) measurement.
        SensorState<Location> location;
        //! GPS altitude measurement.
        SensorState<int32_t> altitude;

        //! Check whether the configuration stored in the VCU is known.
        void checkStoredConfiguration() {
            configuration_available
                = steering_servo.isConfigurationAvailable() && esc.isConfigurationAvailable()
                  && gearbox_servo.isConfigurationAvailable() && radio.isConfigurationAvailable();
        }

      public:
        //! Constructor.
        /*!
         *  \param steering_servo_defaults Default settings for the steering servo.
         *  \param esc_defaults Default settings for the ESC or throttle servo.
         *  \param gearbox_servo_defaults Default settings for the gearbox servo.
         *  \param radio_defaults Default settings for the HMI and VCU radios.
         */
        CarState(ServoConfigParams steering_servo_defaults, ServoConfigParams esc_defaults,
                 GearboxServoConfigParams<0, 1, 2> gearbox_servo_defaults,
                 RadioConfigParams radio_defaults);

        //! Default destructor.
        ~CarState() = default;

        //! Initialize the car state.
        void setup() { ; }

        //! Spinning loop.
        void spinOnce() { ; }

        //! Check whether the configuration stored in the VCU is available.
        /*!
         *  \return True if the (full) configuration is received from the VCU, false if not.
         */
        bool isConfigurationAvailable() { return configuration_available; }

        //! Check whether the current configuration is sent to the VCU.
        /*!
         *  \return True if the current configuration is sent to the VCU.
         */
        bool isCurrentConfigurationStored() {
            return steering_servo.isCurrentConfigurationStored()
                   && esc.isCurrentConfigurationStored()
                   && gearbox_servo.isCurrentConfigurationStored()
                   && radio.isCurrentConfigurationStored();
        }

        //! Ask to save the current configuration.
        void requestConfigurationSave() {
            if(configuration_available)  // Prevent overwriting the configuration before the VCU
                                         // configuration is known.
            {
                configuration_save_request = true;
            }
        }

        //! Check whether there is a request to save the configuration.
        /*!
         *  \return True if the the configuration should be saved to non-volatile memory.
         */
        bool isConfigurationSaveRequested() { return configuration_save_request; }

        //! Reset the save configuration flag.
        void configurationSaved() { configuration_save_request = false; }

        //! Set the steering input.
        /*!
         *  \param steering Steering input (-1000...0...+1000).
         */
        void setSteeringInput(int16_t steering);

        //! Set the steering configuration stored in the VCU.
        /*!
         *  \param stored_configuration Steering configuration stored in the VCU.
         */
        void setStoredSteeringConfiguration(const ServoConfigParams &stored_configuration) {
            steering_servo.setStoredConfiguration(stored_configuration, true);
            checkStoredConfiguration();
        }

        //! Check whether the steering configuration stored in the VCU is available in the HMI.
        /*!
         *  \return True if the steering configuration stored in the VCU is available, false if not.
         */
        bool isSteeringConfigurationAvailable() {
            return steering_servo.isConfigurationAvailable();
        }

        //! Get the current steering configuration.
        /*!
         *  \return The current (stored in the VCU or modified) steering configuration.
         */
        const ServoConfigParams &getSteeringConfiguration() {
            return steering_servo.getCurrentConfiguration();
        }

        //! Check whether all steering configuration changes in the HMI are stored in the VCU.
        /*!
         *  \return True if all steering configuration changes in the HMI are stored in the VCU.
         */
        bool isCurrentSteeringConfigurationStored() {
            return steering_servo.isCurrentConfigurationStored();
        }

        //! Mark the current steering configuration as stored in the VCU.
        void steeringConfigurationStored() { steering_servo.setCurrentConfigurationStored(); }

        //! Set the physical input channel used for steering.
        /*!
         *  \param input_channel Physical input channel used for steering.
         */
        void setSteeringInputChannel(uint8_t input_channel);

        //! Set the pulse width to steer fully to the left.
        /*!
         *  \param pulse_width Pulse width (in µs) to steer fully to the left.
         */
        void setSteerLeftPulseWidth(uint16_t pulse_width);

        //! Set the pulse width to center the wheels.
        /*!
         *  \param pulse_width Pulse width (in µs) to center the wheels.
         */
        void setSteerCenterPulseWidth(uint16_t pulse_width);

        //! Set the pulse width to steer fully to the right.
        /*!
         *  \param pulse_width Pulse width (in µs) to steer fully to the right.
         */
        void setSteerRightPulseWidth(uint16_t pulse_width);

        //! Get the current (normalized) steering position.
        /*!
         *  \return The current normalized (-1000...0...1000) steering position.
         */
        int16_t getCurrentSteeringPosition() { return steering_servo.getCurrentPosition(); }

        //! Set the throttle input.
        /*!
         *  \param throttle Throttle input (-1000...0...+1000).
         */
        void setThrottleInput(int16_t throttle);

        //! Set the throttle configuration stored in the VCU.
        /*!
         *  \param stored_configuration Throttle configuration stored in the VCU.
         */
        void setStoredThrottleConfiguration(const ServoConfigParams &stored_configuration) {
            esc.setStoredConfiguration(stored_configuration, true);
            checkStoredConfiguration();
        }

        //! Check whether the throttle configuration stored in the VCU is available in the HMI.
        /*!
         *  \return True if the throttle configuration stored in the VCU is available, false if not.
         */
        bool isThrottleConfigurationAvailable() { return esc.isConfigurationAvailable(); }

        //! Get the current throttle configuration.
        /*!
         *  \return The current (stored in the VCU or modified) throttle configuration.
         */
        const ServoConfigParams &getThrottleConfiguration() {
            return esc.getCurrentConfiguration();
        }

        //! Check whether all throttle configuration changes in the HMI are stored in the VCU.
        /*!
         *  \return True if all throttle configuration changes in the HMI are stored in the VCU.
         */
        bool isCurrentThrottleConfigurationStored() { return esc.isCurrentConfigurationStored(); }

        //! Mark the current throttle configuration as stored in the VCU.
        void throttleConfigurationStored() { esc.setCurrentConfigurationStored(); }

        //! Set the physical input channel used for the throttle.
        /*!
         *  \param input_channel Physical input channel used for the throttle.
         */
        void setThrottleInputChannel(uint8_t input_channel);

        //! Set the pulse width to drive full speed backwards.
        /*!
         *  \param pulse_width Pulse width (in µs) to drive full speed backwards.
         */
        void setFullBackwardsPulseWidth(uint16_t pulse_width);

        //! Set the pulse width to stop driving.
        /*!
         *  \param pulse_width Pulse width (in µs) to stop driving.
         */
        void setStopPulseWidth(uint16_t pulse_width);

        //! Set the pulse width to drive full speed forward.
        /*!
         *  \param pulse_width Pulse width (in µs) to drive full speed forward.
         */
        void setFullForwardPulseWidth(uint16_t pulse_width);

        //! Get the current (normalized) throttle position.
        /*!
         *  \return The current normalized (-1000...0...+1000) throttle position.
         */
        int16_t getCurrentThrottlePosition() { return esc.getCurrentPosition(); }

        //! Set the gearbox input.
        /*!
         *  \param gearbox Gearbox input (-1000...0...+1000).
         */
        void setGearboxInput(int16_t gearbox);

        //! Update the gearbox configuration stored in the VCU.
        /*!
         *  \param stored_configuration Gearbox configuration stored in the VCU.
         *  \param complete True if the gearbox configuration is complete, false if it's still
         * incomplete.
         */
        void
        setStoredGearboxConfiguration(const GearboxServoConfigParams<0, 1, 2> &stored_configuration,
                                      bool complete) {
            gearbox_servo.setStoredConfiguration(stored_configuration, complete);
            checkStoredConfiguration();
        }

        //! Check whether the gearbox configuration stored in the VCU is available in the HMI.
        /*!
         *  \return True if the gearbox configuration stored in the VCU is available, false if not.
         */
        bool isGearboxConfigurationAvailable() { return gearbox_servo.isConfigurationAvailable(); }

        //! Get the current gearbox configuration.
        /*!
         *  \return The current (stored in the VCU or modified) gearbox configuration.
         */
        const GearboxServoConfigParams<0, 1, 2> &getGearboxConfiguration() {
            return gearbox_servo.getCurrentConfiguration();
        }

        //! Check whether all gearbox configuration changes in the HMI are stored in the VCU.
        /*!
         *  \return True if all gearbox configuration changes in the HMI are stored in the VCU.
         */
        bool isCurrentGearboxConfigurationStored() {
            return gearbox_servo.isCurrentConfigurationStored();
        }

        //! Mark the current gearbox configuration as stored in the VCU.
        void gearboxConfigurationStored() { gearbox_servo.setCurrentConfigurationStored(); }

        //! Set the physical input channel used to shift.
        /*!
         *  \param input_channel Physical input channel used to shift.
         */
        void setGearboxInputChannel(uint8_t input_channel);

        //! Set the pulse width for a specific gear.
        /*!
         *  If the specified gear doesn't exist, the command is silently ignored.
         *
         *  \param gear Gear.
         *  \param pulse_width Pulse width (in µs) for the specified gear.
         */
        void setGearPulseWidth(int8_t gear, uint16_t pulse_width);

        //! Get the current gear.
        /*!
         *  \return The current gear.
         */
        int8_t getCurrentGear() { return gearbox_servo.getCurrentGear(); }

        //! Update the radio configuration stored in the VCU.
        /*!
         *  \param stored_configuration Radio configuration stored in the VCU.
         */
        void setStoredRadioConfiguration(const RadioConfigParams &stored_configuration) {
            radio.setStoredConfiguration(stored_configuration);
            checkStoredConfiguration();
        }

        //! Check whether the radio configuration stored in the VCU is available in the HMI.
        /*!
         *  \return True if the radio configuration stored in the VCU is available, false if not.
         */
        bool isRadioConfigurationAvailable() { return radio.isConfigurationAvailable(); }

        //! Get the current radio configuration.
        /*!
         *  \return The current (stored in the VCU or modified) radio configuration.
         */
        const RadioConfigParams &getRadioConfiguration() { return radio.getCurrentConfiguration(); }

        //! Check whether all radio configuration changes in the HMI are stored in the VCU.
        /*!
         *  \return True if all radio configuration changes in the HMI are stored in the VCU.
         */
        bool isCurrentRadioConfigurationStored() { return radio.isCurrentConfigurationStored(); }

        //! Mark the current radio configuration as stored in the VCU.
        void radioConfigurationStored() { radio.setCurrentConfigurationStored(); }

        //! Set the HMI radio's power level.
        /*!
         *  \param power_level The HMI radio's power level.
         */
        void setHmiRadioPower(uint8_t power_level);

        //! Set the VCU radio's power level.
        /*!
         *  \param power_level The VCU radio's power level.
         */
        void setVcuRadioPower(uint8_t power_level);

        //! Set right turn signal input.
        /*!
         *  \param state Right turn signal input.
         */
        void setTurnSignalRight(bool state);

        //! Check whether the right turn signal is cancelled by steering inputs.
        /*!
         *  \param reset True to reset the cancelled flag, false to keep the current state of the
         * cancelled flag. \return True if the turn signal was cancelled since the last time this
         * function was called with reset = true).
         */
        bool getTurnSignalRightCancelled(bool reset = true);

        //! Set left turn signal input.
        /*!
         *  \param state Left turn signal input.
         */
        void setTurnSignalLeft(bool state);

        //! Check whether the left turn signal is cancelled by steering inputs.
        /*!
         *  \param reset True to reset the cancelled flag, false to keep the current state of the
         * cancelled flag. \return True if the turn signal was cancelled since the last time this
         * function was called with reset = true).
         */
        bool getTurnSignalLeftCancelled(bool reset = true);

        //! Set the dipped beam headlamps input.
        /*!
         *  \param state Dipped beam headlamps input.
         */
        void setDippedBeam(bool state);

        //! Set the high beam headlamps input.
        /*!
         *  \param state High beam headlamps input.
         */
        void setHighBeam(bool state);

        //! Set the hazard flashers input.
        /*!
         *  \param state Hazard flashers input.
         */
        void setHazardFlashers(bool state);

        //! Get requested automotive lighting state.
        /*!
         *  \return Requested automotive lighting state to communicate to the VCU.
         */
        const Lighting &getRequestedLightingState() { return lighting; }

        //! Set speedometer, odometer and trip meter measurements.
        /*!
         *  \param value Measurements.
         *  \param valid True if the last measurements are valid, false of not.
         */
        void setSpeedometer(const Speedometer &value, bool valid = true) {
            speedometer.set(value, valid);
        }

        //! Check whether the speedometer, odometer and tripmeter are updated.
        /*!
         *  Check whether the speedometer, odometer or tripmeter are updated since the last
         * getSpeedometer(reset = true) call.
         *
         *  \return True if the speedometer, odometer or trip meter were updated, false if not.
         */
        bool isSpeedometerUpdated() { return speedometer.isUpdated(); }

        //! Get the current speedometer, odometer and trip meter measurements.
        /*!
         *  \param reset_updated True to reset the updated flag, false to keep it.
         *  \return A pair with a flag indicating whether the current measurements are valid and the
         * measurements itself.
         */
        auto getSpeedometer(bool reset_updated = true) { return speedometer.get(reset_updated); }

        //! Set orientation measurements.
        /*!
         *  \param value Measurements.
         *  \param valid True if the last measurements are valid, false of not.
         */
        void setOrientation(const Orientation &value, bool valid = true) {
            orientation.set(value, valid);
        }

        //! Check whether the orientation is updated.
        /*!
         *  Check whether the orientation is updated since the last getOrientation(reset = true)
         * call.
         *
         *  \return True if the orientation was updated, false if not.
         */
        bool isOrientationUpdated() { return orientation.isUpdated(); }

        //! Get the current orientation measurements.
        /*!
         *  \param reset_updated True to reset the updated flag, false to keep it.
         *  \return A pair with a flag indicating whether the current measurements are valid and the
         * measurements itself.
         */
        auto getOrientation(bool reset_updated = true) { return orientation.get(reset_updated); }

        //! Set GPS location (latitude, longitude) measurements.
        /*!
         *  \param value Measurements.
         *  \param valid True if the last measurements are valid, false of not.
         */
        void setLocation(const Location &value, bool valid = true) { location.set(value, valid); }

        //! Check whether the GPS location (latitude, longitude) is updated.
        /*!
         *  Check whether the GPS location is updated since the last getLocation(reset = true) call.
         *
         *  \return True if the GPS location was updated, false if not.
         */
        bool isLocationUpdated() { return location.isUpdated(); }

        //! Get the current GPS location (latitude, longitude) measurements.
        /*!
         *  \param reset_updated True to reset the updated flag, false to keep it.
         *  \return A pair with a flag indicating whether the current measurements are valid and the
         * measurements itself.
         */
        auto getLocation(bool reset_updated = true) { return location.get(reset_updated); }

        //! Set GPS altitude measurement.
        /*!
         *  \param value Measurement.
         *  \param valid True if the last measurement is valid, false of not.
         */
        void setAltitude(int32_t value, bool valid = true) { altitude.set(value, true); }

        //! Check whether the GPS altitude is updated.
        /*!
         *  Check whether the GPS altitude is updated since the last getAltitude(reset = true) call.
         *
         *  \return True if the GPS altitude was updated, false if not.
         */
        bool isAltitudeUpdated() { return altitude.isUpdated(); }

        //! Get the current GPS altitude measurement.
        /*!
         *  \param reset_updated True to reset the updated flag, false to keep it.
         *  \return A pair with a flag indicating whether the current measurement is valid and the
         * measurement itself.
         */
        auto getAltitude(bool reset_updated = true) { return altitude.get(reset_updated); }
    };

}  // namespace earth_rover_hmi

#endif