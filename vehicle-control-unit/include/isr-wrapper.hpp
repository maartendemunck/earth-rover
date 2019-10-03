#ifndef __ISR_WRAPPER__
#define __ISR_WRAPPER__


#include <functional>


namespace earth_rover
{

  template<uint8_t irq>
  class ISRWrapper
  {

    private:

      bool our_handler_active;
      static std::function<void()> isr;

    public:

      ISRWrapper()
      :
        our_handler_active {false}
      {
        ;
      }

      ~ISRWrapper()
      {
        detachISR();  // detachISR function checks whether our handler is active or not.
      }

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