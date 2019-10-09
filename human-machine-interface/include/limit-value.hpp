//! Limit values to a specified interval (template implementation).
/*!
 *  \ingroup HMI
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_HMI__LIMIT_VALUE__
#define __EARTH_ROVER_HMI__LIMIT_VALUE__


namespace earth_rover_hmi
{

  //! Limit a value to a given interval.
  /*!
   *  \param value Value to limit.
   *  \param lower Lower bound.
   *  \param upper Upper bound.
   *  \return Bounded value.
   * 
   *  \ingroup HMI
   */
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