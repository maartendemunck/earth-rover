//! Wrapper to use any std::function objects as an ISR (interface and template implementation).
/*!
 *  \ingroup VCU
 *  \file
 *  \author Maarten De Munck <maarten@vijfendertig.be>
 */


#ifndef __EARTH_ROVER_VCU__ISR_WRAPPER__
#define __EARTH_ROVER_VCU__ISR_WRAPPER__


#include <functional>


namespace earth_rover_vcu
{

  //! Wrapper to use any std::function object as an ISR (intterupt service routine).
  /*!
   *  Of course, the general ISR programming guidelines still apply: they should be as short and fast as possible.
   * 
   *  \tparam irq Interrupt number.
   */
  template<uint8_t irq>
  class ISRWrapper
  {

    private:

      bool our_handler_active;           //!< True if our ISR is active.
      static std::function<void()> isr;  //!< Active ISR for our interrupt.

    public:

      //! Constructor.
      ISRWrapper()
      :
        our_handler_active {false}
      {
        ;
      }

      //! Destructor.
      /*!
       *  If our interrupt handler is still active, disable it.
       */
      ~ISRWrapper()
      {
        detachISR();  // detachISR function checks whether our handler is active or not.
      }

      //! Attach an ISR (if no ISR is attached yet).
      /*!
       *  \param new_isr ISR as an std::function object.
       *  \param mode When to trigger the interrupt.
       *  \return True if the new ISR is attached, false if another ISR was attached before.
       */
      bool attachISR(std::function<void()> new_isr, int mode)
      {
        bool attached_isr {false};
        if(!isr)
        {
          our_handler_active = true;
          isr = new_isr;
          attachInterrupt(irq, &ISRWrapper<irq>::ISR, mode);
          attached_isr = true;
        }
        // Return true (success) if our new handler is active or false (failure) if another handler was active.
        return attached_isr;
      }

      //! Detach our ISR (if our ISR is attached).
      /*!
       *  \return True if our handler is detached, false if another ISR is attached.
       */
      bool detachISR()
      {
        if(our_handler_active)
        {
          detachInterrupt(irq);
          isr = std::function<void()>();
          our_handler_active = false;
        }
        // Return true (succes) if no handler is active or false (failure) is another object's handler is active.
        return !isr;
      }

      //! Static ISR function.
      /*!
       *  This function calls the std::function object currently attached to this interrupt.
       */
      static void ISR()
      {
        if(isr)
        {
          isr();
        }
      }

  };


  template<uint8_t irq>
  std::function<void()> ISRWrapper<irq>::isr{};

}

#endif