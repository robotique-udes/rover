#ifndef PWM_GENERATOR_HPP
#define PWM_GENERATOR_HPP

#include "rover_lib2/rover_object.hpp"

namespace PWMGenerators
{

    /**
     * @brief Template shadowing for type validation
     *
     */
    class PWMGeneratorT
    {
      protected:
        PWMGeneratorT() = default;
    };

    /**
     * @brief RTCP Interface
     *
     * @tparam Impl_T
     */
    template<typename Impl_T>
    class PWMGenerator : public PWMGeneratorT,
                         public RoverObject<PWMGenerator<Impl_T>>
    {
      private:
        friend Impl_T;
        PWMGenerator() = default;

      public:
        void _init(void)
        {
            static_cast<Impl_T*>(this)->__init();
        }

        void _update(void)
        {
            static_cast<Impl_T*>(this)->__update();
        }

        /**
         * @brief Range is [0.0F; 100.0F]
         *
         * @param duty_
         */
        void setDutyCycle(float duty_)
        {
            static_cast<Impl_T*>(this)->_setDutyCycle(duty_);
        }

        /**
         * @brief Range is [0.0F; 100.0F]
         *
         */
        float getDutyCycle(void) const
        {
            return static_cast<const Impl_T*>(this)->_getDutyCycle();
        }

        /**
         * @brief Changing frequency is expensive and will often yield in jerky PWM signals when transitionning
         *
         * @param duty_
         */
        void setFrequency(float frequency_)
        {
            static_cast<Impl_T*>(this)->_setFrequency(frequency_);
        }

        float getFrequency(void) const
        {
            return static_cast<const Impl_T*>(this)->_getFrequency();
        }
    };

}  // namespace PWMGenerators

#endif  // PWM_GENERATOR_HPP
