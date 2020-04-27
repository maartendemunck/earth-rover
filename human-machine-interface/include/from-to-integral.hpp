//! Convert enum types to their underlying integral types and vice versa (template implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */

#ifndef __EARTH_ROVER_HMI__FROM_TO_INTEGRAL__
#define __EARTH_ROVER_HMI__FROM_TO_INTEGRAL__

#include <type_traits>

namespace earth_rover_hmi {

    //! Convert an integral type to an enum type.
    /*!
     *  \param value Integral value.
     *  \return Corresponding enum value.
     *
     *  \ingroup HMI
     */
    template <typename Enum>
    constexpr Enum from_integral(typename std::underlying_type<Enum>::type value) {
        return static_cast<Enum>(value);
    }

    //! Convert an enum type to its underlying integral type.
    /*!
     *  \param value Enum value.
     *  \return Corresponding integral value.
     *
     *  \ingroup HMI
     */
    template <typename Enum>
    constexpr typename std::underlying_type<Enum>::type to_integral(Enum value) {
        return static_cast<typename std::underlying_type<Enum>::type>(value);
    }

}  // namespace earth_rover_hmi

#endif