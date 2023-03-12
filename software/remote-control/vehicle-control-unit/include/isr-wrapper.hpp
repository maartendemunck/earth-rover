// Copyright 2019-2023 Vijfendertig BV.
//
// This file is part of the Earth Rover project, which is licensed under the 3-Clause BSD License.
// See the file LICENSE.md or go to https://opensource.org/license/bsd-3-clause/ for full license
// details.

#ifndef __EARTH_ROVER__ISR_WRAPPER__
#define __EARTH_ROVER__ISR_WRAPPER__

#include <functional>

namespace earth_rover {

    template <uint8_t irq> class ISRWrapper {

      private:
        bool our_handler_active;
        static std::function<void()> isr;

      public:
        ISRWrapper() : our_handler_active{false} {
            ;
        }

        ~ISRWrapper() {
            detachISR();
        }

        bool attachISR(std::function<void()> new_isr, int mode) {
            bool attached_isr{false};
            if(!isr) {
                our_handler_active = true;
                isr = new_isr;
                attachInterrupt(irq, &ISRWrapper<irq>::ISR, mode);
                attached_isr = true;
            }
            bool error = !attached_isr;
            return error;
        }

        bool detachISR() {
            if(our_handler_active) {
                detachInterrupt(irq);
                isr = std::function<void()>();
                our_handler_active = false;
            }
            bool error = !isr;
            return error;
        }

        static void ISR() {
            if(isr) {
                isr();
            }
        }
    };

    template <uint8_t irq> std::function<void()> ISRWrapper<irq>::isr{};

}  // namespace earth_rover

#endif