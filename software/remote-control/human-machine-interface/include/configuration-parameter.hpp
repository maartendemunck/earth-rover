// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__CONFIGURATION_PARAMETER__
#define __EARTH_ROVER__CONFIGURATION_PARAMETER__

namespace earth_rover {

    template <typename ConfigParameter_t> class ConfigParameter {

      private:
        ConfigParameter_t current_value;
        bool saved_value_is_available;
        bool is_sent_to_vcu;

      public:
        ConfigParameter() : current_value{}, saved_value_is_available{false}, is_sent_to_vcu{true} {
            ;
        }

        explicit ConfigParameter(ConfigParameter_t default_value,
                                 bool saved_value_is_available = false)
            : current_value{default_value}, saved_value_is_available{saved_value_is_available},
              is_sent_to_vcu{true} {
            ;
        }

        void setSavedConfig(const ConfigParameter_t &new_value,
                            bool saved_value_is_complete = true) {
            current_value = new_value;
            saved_value_is_available = saved_value_is_complete;
        }

        bool isConfigAvailable() {
            return saved_value_is_available;
        }

        void setCurrentConfig(ConfigParameter_t new_value) {
            if(new_value != current_value) {
                is_sent_to_vcu = false;
                current_value = new_value;
            }
        }

        const ConfigParameter_t &getCurrentConfig() {
            return current_value;
        }

        bool isCurrentConfigSentToVcu() {
            return is_sent_to_vcu;
        }

        void markCurrentConfigSentToVcu() {
            is_sent_to_vcu = true;
        }
    };

}  // namespace earth_rover

#endif