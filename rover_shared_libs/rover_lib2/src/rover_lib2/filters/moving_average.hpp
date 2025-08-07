#ifndef ROVER_LIB2_FILTERS_MOVING_AVERAGE_HPP
#define ROVER_LIB2_FILTERS_MOVING_AVERAGE_HPP

#include "rover_lib2/filters/filter.hpp"
#include "rover_lib2/helpers/macros.hpp"

#include <cstdint>

namespace Filters
{
    template<class T, uint16_t COEFF_NB>
    class MovingAverage
    {
      public:
        MovingAverage(T startingValue = static_cast<T>(0))
        {
            reset(startingValue);
        }

        ~MovingAverage() = default;

        float addValue(T value)
        {
            _avg -= static_cast<float>(_avgTable[_cursor]);
            _avg += static_cast<float>(value);
            _avgTable[_cursor] = value;
            _cursor = (_cursor + 1U) % COEFF_NB;
            return getFilteredValue();
        }

        float getFilteredValue() const
        {
            return _avg / static_cast<float>(COEFF_NB);
        }

        void reset(T fillValue_)
        {
            for (uint16_t i = 0U; i < COEFF_NB; ++i)
            {
                _avgTable[i] = fillValue_;
            }
            _avg = fillValue_ * COEFF_NB;
            _cursor = 0U;
        }

      private:
        T _avgTable[COEFF_NB] = {};
        uint16_t _cursor = 0U;
        float _avg = 0.0F;

        VALIDATE_CONCEPT(Filter, MovingAverage);
    };
}  // namespace Filters

#endif  // ROVER_LIB2_FILTERS_MOVING_AVERAGE_HPP
