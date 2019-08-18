#ifndef __TO_INTEGRAL__
#define __TO_INTEGRAL__


#include <type_traits>


namespace earth_rover
{

  template<typename Enum>
  constexpr Enum from_integral(typename std::underlying_type<Enum>::type value)
  {
    return static_cast<Enum>(value);
  }

  template<typename Enum>
  constexpr typename std::underlying_type<Enum>::type to_integral(Enum value)
  {
    return static_cast<typename std::underlying_type<Enum>::type>(value);
  }

}

#endif