#ifndef __TO_INTEGRAL__
#define __TO_INTEGRAL__


#include <type_traits>


namespace earth_rover
{

  template<typename Enum>
  constexpr typename std::underlying_type<Enum>::type to_integral(Enum value)
  {
    return static_cast<typename std::underlying_type<Enum>::type>(value);
  }

}

#endif