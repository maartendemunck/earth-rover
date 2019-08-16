#ifndef __LIMIT_VALUE__
#define __LIMIT_VALUE__


namespace earth_rover
{

  template<typename T>
  T limit_value(T value, T lower, T upper)
  {
    if(value < lower)
    {
      return lower;
    }
    else if(value > upper)
    {
      return upper;
    }
    else {
      return value;
    }
  }

}

#endif