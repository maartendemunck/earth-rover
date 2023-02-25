// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__MESSAGES__
#define __EARTH_ROVER__MESSAGES__

#include <cstdint>

namespace earth_rover {

    enum class RequestMessageType : uint8_t {
        Control = 0x00,
        RequestState = 0x10,
        RequestConfig = 0x30,
        ConfigureSteeringServo = 0x20,
        ConfigureEsc = 0x21,
        ConfigureGearboxServo = 0x22,
        ConfigureRadio = 0x24,
        SaveConfig = 0x2f
    };

    enum class RequestStateIDs : uint8_t {
        SpeedAndOdometer = 0x01,
        Orientation = 0x02,
        Location = 0x04,
        Altitude = 0x08
    };

    enum class RequestConfigIDs : uint8_t {
        Steering = 0x01,
        Throttle = 0x02,
        Gearbox = 0x04,
        Radio = 0x80
    };

    enum class ResponseMessageType : uint8_t {
        Speedometer = 0x90,
        Orientation = 0x91,
        Location = 0x92,
        Altitude = 0x93,
        SteeringServoConfig = 0xb0,
        EscConfig = 0xb1,
        GearboxServoConfig = 0xb2,
        RadioConfig = 0xb4
    };

}  // namespace earth_rover

#endif