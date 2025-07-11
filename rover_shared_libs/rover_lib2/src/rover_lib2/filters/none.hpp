#ifndef ROVER_LIB2_FILTERS_NONE_HPP
#define ROVER_LIB2_FILTERS_NONE_HPP

#include "filter.hpp"

namespace Filters
{

    /**
     * @brief Returns the stored value, allow sensors to not be filtered with limited overhead
     *
     */
    class None
    {
      public:
        float addValue(float value_)
        {
            _value = value_;
            return this->getFilteredValue();
        }

        float getFilteredValue() const
        {
            return _value;
        }

        void reset(float fillValue_)
        {
            _value = fillValue_;
        }

      private:
        float _value;

        VALIDATE_CONCEPT(Filter, None);
    };

}  // namespace Filters

#endif  // ROVER_LIB2_FILTERS_NONE_HPP
