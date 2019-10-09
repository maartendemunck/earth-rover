#ifndef __EARTH_ROVER_HMI__CONFIGURATION__
#define __EARTH_ROVER_HMI__CONFIGURATION__


namespace earth_rover_hmi
{

  class Configuration
  {
    public:

      enum class Changed { Configuration, Display };

    protected:

      bool configuration_changed;
      bool display_changed;

    public:

      inline Configuration():
        configuration_changed {false},
        display_changed {false}
      {
        ;
      }

      ~Configuration() = default;

      inline bool isConfigurationChanged()
      {
        return configuration_changed;
      };

      inline bool isDisplayChanged()
      {
        return display_changed;
      };

    protected:

      inline void updateChanged(Configuration::Changed change_source)
      {
        if(change_source == Configuration::Changed::Configuration)
        {
          configuration_changed = true;
        }
        else if(change_source == Configuration::Changed::Display)
        {
          display_changed = true;
        }
      }

  };

}

#endif