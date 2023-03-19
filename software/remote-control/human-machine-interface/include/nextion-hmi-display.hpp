// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__NEXTION_HMI_DISPLAY__
#define __EARTH_ROVER__NEXTION_HMI_DISPLAY__

#include "car-state.hpp"
#include "from-to-integral.hpp"
#include "hmi-state.hpp"
#include "limit-value.hpp"

namespace earth_rover {

    template <typename SerialDevice_t> class NextionHmiDisplay {

      private:
        enum class HmiPage : uint8_t {
            Speedometer = 0x01u,
            Orientation = 0x02u,
            Location = 0x03u,
            SteeringSettings = 0x11u,
            ThrottleSettings = 0x12u,
            GearboxSettings = 0x13u,
            RadioSettings = 0x14u
        };

        enum class AutomotiveLight : uint8_t {
            TurnSignalRight = 0x01u,
            TurnSignalLeft = 0x02u,
            DippedBeam = 0x03u,
            HighBeam = 0x04u,
            HazardFlashers = 0x05u
        };

        SerialDevice_t &serial_device;
        static constexpr uint16_t rx_buffer_size{8};
        uint8_t rx_buffer[rx_buffer_size];
        uint16_t rx_buffer_pointer;

        CarState &car_state;
        HmiState &hmi_state;

        HmiPage current_page;
        bool config_pages_available;
        bool display_config_complete;

      public:
        NextionHmiDisplay(SerialDevice_t &serial_device, CarState &car_state, HmiState &hmi_state)
            : serial_device{serial_device}, rx_buffer_pointer{0u}, car_state{car_state},
              hmi_state{hmi_state}, current_page{HmiPage::Speedometer},
              config_pages_available{false}, display_config_complete{false} {
            ;
        }

        ~NextionHmiDisplay() {
            serial_device.flush();
            serial_device.end();
        }

        void setup() {
            serial_device.begin(115200u);
            while(!serial_device) {
                ;
            }
            serial_device.print("\xff\xff\xff");
            serial_device.print("bkcmd=0\xff\xff\xff");
            disableConfigPages();
            updateSettingsOnDisplay(true);
            serial_device.print("page speedometer\xff\xff\xff");
            current_page = HmiPage::Speedometer;
            serial_device.flush();
            delay(50);
            display_config_complete = true;
        }

        void spinOnce() {
            if(!config_pages_available && car_state.isConfigAvailable()) {
                enableConfigPages();
            }
            if(car_state.getTurnSignalRightCancelled(true)) {
                setTurnSignalRight(false);
            }
            if(car_state.getTurnSignalLeftCancelled(true)) {
                setTurnSignalLeft(false);
            }
            updateSettingsOnDisplay();
        }

        void receiveData() {
            if(!display_config_complete) {
                return;
            }
            while(serial_device.available()) {
                rx_buffer[rx_buffer_pointer++] = char(serial_device.read() & 0x00ffu);
                if(rx_buffer_pointer >= 3 && rx_buffer[rx_buffer_pointer - 1] == 0xffu
                   && rx_buffer[rx_buffer_pointer - 2] == 0xffu
                   && rx_buffer[rx_buffer_pointer - 3] == 0xffu) {
                    if(rx_buffer[0] == 0xa0u && rx_buffer_pointer == 5) {
                        processPageChangeCommand(rx_buffer);
                    }
                    else if(rx_buffer[0] == 0xa1u && rx_buffer_pointer == 6) {
                        processLightingCommand(rx_buffer);
                    }
                    else if(rx_buffer[0] == 0xa2u && rx_buffer_pointer == 8) {
                        processConfigCommand(rx_buffer);
                    }
                    else if(rx_buffer[0] == 0xa3u && rx_buffer_pointer == 6) {
                        processStickCalibrationCommand(rx_buffer);
                    }
                    rx_buffer_pointer = 0;
                }
                // Prevent buffer overflows.
                if(rx_buffer_pointer >= rx_buffer_size) {
                    rx_buffer_pointer = rx_buffer_size - 1;
                    for(uint16_t i = 1; i < rx_buffer_size; ++i) {
                        rx_buffer[i - 1] = rx_buffer[i];
                    }
                }
            }
        }

      private:
        void processPageChangeCommand(uint8_t buffer[rx_buffer_size]) {
            static_assert(rx_buffer_size >= 5, "page change command is 5 bytes");
            auto new_page = from_integral<HmiPage>(buffer[1]);
            if(new_page != current_page) {
                requestConfigSaveIfNecessary(new_page);
                current_page = new_page;
                updateSettingsOnDisplay(true);
            }
        }

        void requestConfigSaveIfNecessary(HmiPage new_page) {
            if((current_page == HmiPage::SteeringSettings
                || current_page == HmiPage::ThrottleSettings
                || current_page == HmiPage::GearboxSettings
                || current_page == HmiPage::RadioSettings)
               && (new_page == HmiPage::Speedometer || new_page == HmiPage::Orientation
                   || new_page == HmiPage::Location)) {
                hmi_state.requestConfigSave();
                car_state.requestConfigSaveByVcu();
            }
        }

        void processLightingCommand(uint8_t buffer[rx_buffer_size]) {
            static_assert(rx_buffer_size >= 6, "lighting command is 6 bytes");
            AutomotiveLight light = from_integral<AutomotiveLight>(buffer[1]);
            bool state = buffer[2];
            switch(light) {
                case AutomotiveLight::TurnSignalRight:
                    car_state.setTurnSignalRight(state);
                    break;
                case AutomotiveLight::TurnSignalLeft:
                    car_state.setTurnSignalLeft(state);
                    break;
                case AutomotiveLight::DippedBeam:
                    car_state.setDippedBeam(state);
                    break;
                case AutomotiveLight::HighBeam:
                    car_state.setHighBeam(state);
                    break;
                case AutomotiveLight::HazardFlashers:
                    car_state.setHazardFlashers(state);
                    break;
            }
        }

        void processConfigCommand(uint8_t buffer[rx_buffer_size]) {
            static_assert(rx_buffer_size >= 8, "config command is 8 bytes");
            HmiPage page = from_integral<HmiPage>(buffer[1]);
            uint8_t setting = buffer[2];
            uint16_t value = (buffer[4] << 8) + buffer[3];
            if(page == HmiPage::SteeringSettings) {
                switch(setting) {
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
            }
            else if(page == HmiPage::ThrottleSettings) {
                switch(setting) {
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
            }
            else if(page == HmiPage::GearboxSettings) {
                switch(setting) {
                    case 1:
                        car_state.setGearPulseWidth(1, value);
                        break;
                    case 3:
                        car_state.setGearPulseWidth(2, value);
                        break;
                }
            }
            else if(page == HmiPage::RadioSettings) {
                switch(setting) {
                    case 1:
                        car_state.setHmiRadioPower(value);
                        break;
                    case 2:
                        car_state.setVcuRadioPower(value);
                        break;
                }
            }
        }

        void processStickCalibrationCommand(uint8_t buffer[rx_buffer_size]) {
            static_assert(rx_buffer_size >= 6, "stick calibration command is 6 bytes");
            // Stick position calibration.
            auto calibration_position{from_integral<AnalogInput::CalibrationPosition>(buffer[2])};
            if(buffer[1] == 0x11) {
                hmi_state.getSteeringInput().calibratePosition(calibration_position);
            }
            else if(buffer[1] == 0x12) {
                hmi_state.getThrottleInput().calibratePosition(calibration_position);
            }
            else if(buffer[1] == 0x13) {
                hmi_state.getGearboxInput().calibratePosition(calibration_position);
            }
        }

        void enableConfigPages() {
            serial_device.print("speedometer.var_settings.val=1\xff\xff\xff");
            serial_device.flush();
            config_pages_available = true;
        }

        void disableConfigPages() {
            serial_device.print("speedometer.var_settings.val=0\xff\xff\xff");
            serial_device.flush();
            config_pages_available = false;
        }

        void setTurnSignalRight(bool state) {
            serial_device.printf("speedometer.var_tsright.val=%d\xff\xff\xff", int16_t(state));
            serial_device.flush();
        }

        void setTurnSignalLeft(bool state) {
            serial_device.printf("speedometer.var_tsleft.val=%d\xff\xff\xff", int16_t(state));
            serial_device.flush();
        }

        void updateSettingsOnDisplay(bool force_update_all = false) {
            if(current_page == HmiPage::Speedometer
               && (force_update_all || car_state.isSpeedometerUpdated())) {
                updateSpeedometerOnDisplay();
            }
            if(current_page == HmiPage::Orientation
               && (force_update_all || car_state.isOrientationUpdated())) {
                updateOrientationOnDisplay();
            }
            if(current_page == HmiPage::Location
               && (force_update_all || car_state.isLocationUpdated())) {
                updateLocationOnDisplay();
            }
            if(current_page == HmiPage::Location
               && (force_update_all || car_state.isAltitudeUpdated())) {
                updateAltitudeOnDisplay();
            }
            if(current_page == HmiPage::SteeringSettings && force_update_all) {
                updateSteeringSettingsOnDisplay();
            }
            if(current_page == HmiPage::ThrottleSettings && force_update_all) {
                updateThrottleSettingsOnDisplay();
            }
            if(current_page == HmiPage::GearboxSettings && force_update_all) {
                updateGearboxSettingsOnDisplay();
            }
            if(current_page == HmiPage::RadioSettings && force_update_all) {
                updateRadioSettingsOnDisplay();
            }
        }

        void updateSpeedometerOnDisplay() {
            auto speedometer = car_state.getSpeedometer();
            if(speedometer.valid == true) {
                uint16_t angle
                    = (320u + uint16_t(26 * limit_value(speedometer.data.speed_kmh, 0., 10.)))
                      % 360u;
                serial_device.printf("z_speed.val=%d\xff\xff\xff", angle);
                uint32_t odometer_raw = int32_t(speedometer.data.odometer_km * 100) % 1000000;
                serial_device.printf("x_odo.val=%d\xff\xff\xff", odometer_raw);
                int32_t tripmeter_raw = int32_t(speedometer.data.tripmeter_km * 1000) % 1000000;
                serial_device.printf("x_trip.val=%d\xff\xff\xff", tripmeter_raw);
            }
            else {
                serial_device.print("z_speed.val=320\xff\xff\xff");
            }
            serial_device.flush();
        }

        void updateOrientationOnDisplay() {
            auto orientation = car_state.getOrientation();
            if(orientation.valid == true) {
                serial_device.printf("z_yaw.val=%d\xff\xff\xff",
                                     (90 + int16_t(orientation.data.yaw_deg)) % 360);
                serial_device.printf(
                    "z_pitch.val=%d\xff\xff\xff",
                    (360 + 90 + 2 * int16_t(limit_value(orientation.data.pitch_deg, -60., 60.)))
                        % 360);
                serial_device.printf(
                    "z_roll.val=%d\xff\xff\xff",
                    (360 + 90 + 2 * int16_t(limit_value(orientation.data.roll_deg, -60., 60.)))
                        % 360);
                serial_device.flush();
            }
        }

        void updateLocationOnDisplay() {
            auto location = car_state.getLocation();
            if(location.valid == true) {
                serial_device.printf(
                    "t_lat.txt=\"%d\xb0%02d'%02d.%02d\\\" %c\"\xff\xff\xff",
                    location.data.latitude.degrees, location.data.latitude.minutes,
                    location.data.latitude.seconds_whole, location.data.latitude.seconds_frac / 100,
                    location.data.latitude.hemisphere == Hemisphere_t::NORTH_H ? 'N' : 'S');
                serial_device.printf(
                    "t_lon.txt=\"%d\xb0%02d'%02d.%02d\\\" %c\"\xff\xff\xff",
                    location.data.longitude.degrees, location.data.longitude.minutes,
                    location.data.longitude.seconds_whole,
                    location.data.longitude.seconds_frac / 100,
                    location.data.longitude.hemisphere == Hemisphere_t::EAST_H ? 'E' : 'W');
            }
            else {
                serial_device.printf("t_lat.txt=\"NO GPS FIX\"\xff\xff\xff");
                serial_device.printf("t_lon.txt=\"NO GPS FIX\"\xff\xff\xff");
            }
            serial_device.flush();
        }

        void updateAltitudeOnDisplay() {
            auto altitude = car_state.getAltitude();
            if(altitude.valid == true) {
                serial_device.printf("t_alt.txt=\"%d.%02dm\"\xff\xff\xff", altitude.data / 100,
                                     abs(altitude.data) % 100);
            }
            else {
                serial_device.printf("t_alt.txt=\"NO GPS FIX\"\xff\xff\xff");
            }

            serial_device.flush();
        }

        void updateSteeringSettingsOnDisplay() {
            auto config = car_state.getSteeringConfig();
            serial_device.printf("set_steering.var_left.val=%u\xff\xff\xff",
                                 config.pulse_width_minimum);
            serial_device.printf("set_steering.var_center.val=%u\xff\xff\xff",
                                 config.pulse_width_center);
            serial_device.printf("set_steering.var_right.val=%u\xff\xff\xff",
                                 config.pulse_width_maximum);
            serial_device.print("page set_steering\xff\xff\xff");
            serial_device.flush();
        }

        void updateThrottleSettingsOnDisplay() {
            auto config = car_state.getThrottleConfig();
            serial_device.printf("set_throttle.var_reverse.val=%u\xff\xff\xff",
                                 config.pulse_width_minimum);
            serial_device.printf("set_throttle.var_stop.val=%u\xff\xff\xff",
                                 config.pulse_width_center);
            serial_device.printf("set_throttle.var_forward.val=%u\xff\xff\xff",
                                 config.pulse_width_maximum);
            serial_device.print("page set_throttle\xff\xff\xff");
            serial_device.flush();
        }

        void updateGearboxSettingsOnDisplay() {
            auto config = car_state.getGearboxConfig();
            serial_device.printf("set_gearbox.var_low.val=%d\xff\xff\xff",
                                 config.getPulseWidthForGear(1));
            serial_device.printf("set_gearbox.var_high.val=%d\xff\xff\xff",
                                 config.getPulseWidthForGear(2));
            serial_device.print("page set_gearbox\xff\xff\xff");
            serial_device.flush();
        }

        void updateRadioSettingsOnDisplay() {
            auto config = car_state.getRadioConfig();
            serial_device.printf("set_radio.var_tx_power.val=%d\xff\xff\xff", config.tx_power);
            serial_device.printf("set_radio.var_rx_power.val=%d\xff\xff\xff", config.rx_power);
            serial_device.print("page set_radio\xff\xff\xff");
            serial_device.flush();
        }
    };

}  // namespace earth_rover

#endif