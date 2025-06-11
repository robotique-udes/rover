#ifndef ROVER_LIB2_LED_BLINK_PATTERN_HPP
#define ROVER_LIB2_LED_BLINK_PATTERN_HPP

#include <cstdint>
#include <limits>
#include <array>
#include <rover_lib2/helpers/compile_time_array.hpp>

namespace LED
{
    static constexpr uint8_t INTENSITY_RESOLUTION = 100U;

    struct BlinkPatternStep
    {
        /**
         * @brief Construct a new Blink Pattern Step object
         *
         * @param duration_ In milliseconds
         * @param intensity_ In percentage/10: [0; 100]
         */
        constexpr BlinkPatternStep(uint32_t durationMs_, uint8_t intensity_):
            durationMs(durationMs_),
            intensity(intensity_)
        {
        }

        BlinkPatternStep() = default;

        uint32_t durationMs;
        uint8_t intensity;
    };

    template<size_t N>
    using BlinkingPattern = CompileTimeArray<BlinkPatternStep, N>;

    namespace BlinkPatterns
    {
        constexpr BlinkingPattern<1> ON = {BlinkPatternStep(std::numeric_limits<uint32_t>::max(), 100U)};
        constexpr BlinkingPattern<1> OFF = {BlinkPatternStep(std::numeric_limits<uint32_t>::max(), 0U)};
        constexpr BlinkingPattern<2> HEARTBEAT = {BlinkPatternStep(500UL, 100U), BlinkPatternStep(500UL, 0U)};
        constexpr BlinkingPattern<4> DOUBLE_FLASH = {BlinkPatternStep(100UL, 100U),
                                                     BlinkPatternStep(100UL, 0U),
                                                     BlinkPatternStep(100UL, 100U),
                                                     BlinkPatternStep(700UL, 0U)};
        constexpr BlinkingPattern<4> ERROR = {BlinkPatternStep(100UL, 10U),
                                              BlinkPatternStep(50UL, 0U),
                                              BlinkPatternStep(100UL, 100U),
                                              BlinkPatternStep(50UL, 0U)};
    };  // namespace BlinkPatterns
}  // namespace LED

#endif  // ROVER_LIB2_LED_BLINK_PATTERN_HPP
