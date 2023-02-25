// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__LIMIT_VALUE__
#define __EARTH_ROVER__LIMIT_VALUE__

namespace earth_rover {

    template <typename T> T limit_value(T value, T lower, T upper) {
        if(value < lower) {
            return lower;
        }
        else if(value > upper) {
            return upper;
        }
        else {
            return value;
        }
    }

}  // namespace earth_rover

#endif