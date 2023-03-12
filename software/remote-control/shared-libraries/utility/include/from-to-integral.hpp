// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__FROM_TO_INTEGRAL__
#define __EARTH_ROVER__FROM_TO_INTEGRAL__

#include <type_traits>

namespace earth_rover {

    template <typename Enum>
    constexpr Enum from_integral(typename std::underlying_type<Enum>::type value) {
        return static_cast<Enum>(value);
    }

    template <typename Enum>
    constexpr typename std::underlying_type<Enum>::type to_integral(Enum value) {
        return static_cast<typename std::underlying_type<Enum>::type>(value);
    }

}  // namespace earth_rover

#endif