// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__RADIO_STATE__
#define __EARTH_ROVER__RADIO_STATE__

#include "configuration-parameter.hpp"
#include "limit-value.hpp"
#include <cstdint>

namespace earth_rover {

    class RadioConfigParams {
      public:
        uint8_t tx_power;
        uint8_t rx_power;

        RadioConfigParams(uint8_t tx_power, uint8_t rx_power)
            : tx_power{tx_power}, rx_power{rx_power} {
            ;
        }

        bool operator!=(const RadioConfigParams &rhs) {
            return (tx_power != rhs.tx_power || rx_power != rhs.rx_power);
        }
    };

    using RadioState = ConfigParameter<RadioConfigParams>;

}  // namespace earth_rover

#endif