#ifndef ROVER_LIB2_FILTERS_LOW_PASS_EMA_HPP
#define ROVER_LIB2_FILTERS_LOW_PASS_EMA_HPP

#include "filter.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/log.hpp"

#include <array>
#include <algorithm>

DEFINE_LOG_NODE(LowPassEMA, Logger::eNodeState::OFF);

namespace Filters
{
    /**
     * @brief Exponential Moving Average (EMA) low-pass filter: y[n] = αx[n] + (1 - α)*y[n-1]
     *
     * @param α (alpha) Smoothing factor [0.0, 1.0]:
     *   - Higher α (→1.0): Less filtering, faster response, more noise
     *   - Lower α (→0.0): More filtering, slower response, less noise
     *
     * Common α values:
     *   - 0.1-0.3: Heavy filtering for noisy sensors
     *   - 0.5-0.7: Moderate filtering for control loops
     *   - 0.8-0.95: Light filtering, fast tracking
     *
     * Cutoff frequency: fc ≈ α*fs/(2π(1-α)) where fs is sampling frequency
     */
    class LowPassEMA
    {
        static constexpr float MAX_ALPHA_VALUE = static_cast<float>(1.0);
        static constexpr float MIN_ALPHA_VALUE = static_cast<float>(0.0);

      public:
        constexpr LowPassEMA(float alpha_ = 0.6F, float initialValue_ = 0.0F):
            _alpha(std::clamp(alpha_, MIN_ALPHA_VALUE, MAX_ALPHA_VALUE)),
            _lastOutput(initialValue_)
        {
            if (alpha_ < MIN_ALPHA_VALUE || alpha_ > MAX_ALPHA_VALUE)
            {
                LOG_WARN(Logger::Nodes::LowPassEMA,
                         "Alpha value (%f) out of range [%f; %f]",
                         alpha_,
                         MIN_ALPHA_VALUE,
                         MAX_ALPHA_VALUE);
            }
        }

        float addValue(float newValue_)
        {
            _lastOutput = _alpha * newValue_ + (static_cast<float>(1.0) - _alpha) * _lastOutput;

            return this->getFilteredValue();
        }

        float getFilteredValue() const
        {
            return _lastOutput;
        }

        void reset(float fillValue_)
        {
            _lastOutput = fillValue_;
        }

      private:
        const float _alpha;
        float _lastOutput;

        VALIDATE_CONCEPT(Filter, LowPassEMA);
    };

}  // namespace Filters

#endif  // ROVER_LIB2_FILTERS_LOW_PASS_EMA_HPP
