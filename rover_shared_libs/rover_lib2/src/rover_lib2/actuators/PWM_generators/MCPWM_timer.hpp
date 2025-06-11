#ifndef MCPWM_TIMER_HPP
#define MCPWM_TIMER_HPP

#include "rover_lib2/helpers/assert.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"

DEFINE_LOG_NODE(MCPWMTimer, Logger::eNodeState::ON);

/**
 * @brief Each timer instance can generate up to two distinct PWM output at the same frequency
 *
 * @note Necessary to create MCPWM Pwm Generator
 */

namespace PWMGenerators
{
    class MCPWMTimer
    {
        friend class MCPWM;

        static constexpr uint32_t CLOCK_FREQUENCY_HZ = 160'000'000UL;
        static constexpr uint32_t MAX_TICK_VALUE = static_cast<uint32_t>(std::numeric_limits<uint16_t>::max());
        static constexpr float MINIMUM_VALID_DUTY_CYCLE_RESOLUTION_PERCENT = 10.0F;

      public:
        static constexpr int DEFAULT_INTERUPT_PRIORITY = 0;

        enum class eMCPWMGroupID : int
        {
            GROUP_0 = 0,
            GROUP_1,
            eLAST
        };
        static_assert(TO_UNDERLYING(eMCPWMGroupID::eLAST) == SOC_MCPWM_GROUPS, "MCU not supported by driver");

        /**
         * @brief
         *
         * @param frequency_ In Hz, Minimum value is 16Hz, use other PWM generator for lower frequencies
         * @param peripheralGroupId_ The esp32-s3 has two independant MCPWM peripheral groups, each can create up to 3 timers.
         * @param timerResolutionHz_ Resolution of the timer's counter. For accurate PWM, the counter should be divisable by the
         * frequency. The effective duty-cycle resolution is timerResolutionHz_/frequency_. The maximum value is the clock
         * source's frequency, on ESP32-S3 it's 160'000'000.
         */
        MCPWMTimer(const uint32_t frequency_, const eMCPWMGroupID peripheralGroupId_ = eMCPWMGroupID::GROUP_1):
            _frequency(1.0F),
            _resolutionHz(MAX_TICK_VALUE - 1),  // Fallback values should never trigger
            _timerPeriodTick(MAX_TICK_VALUE),   // Fallback values
            _groupID(
                CONSTRAIN(peripheralGroupId_, static_cast<eMCPWMGroupID>(0), static_cast<eMCPWMGroupID>(SOC_MCPWM_GROUPS - 1))),
            _enabled(false)
        {
            _frequency = CONSTRAIN(frequency_, 1UL, CLOCK_FREQUENCY_HZ);
            MCPWMTimer::calculateResTickFromFreq(static_cast<uint32_t>(_frequency), _resolutionHz, _timerPeriodTick);

            _frequency = static_cast<float>(_resolutionHz / static_cast<float>(_timerPeriodTick + 1));
            float dutyCycleResolution = 100.0F / (_timerPeriodTick + 1);
            if (_frequency != static_cast<float>(frequency_) || dutyCycleResolution > MINIMUM_VALID_DUTY_CYCLE_RESOLUTION_PERCENT)
            {
                LOG_WARN(Logger::Nodes::MCPWMTimer,
                         "Requested frequency of %u Hz; Optimal possible timings are:\n\tEffective frequency: %.2f "
                         "Hz\n\tDuty-Cycle resolution (steps of): %.3f%%",
                         frequency_,
                         _frequency,
                         dutyCycleResolution);
            }
            else
            {
                LOG_DEBUG(Logger::Nodes::MCPWMTimer,
                          "Requested frequency of %u Hz; Optimal possible timings are:\n\tEffective frequency: %f "
                          "Hz\n\tDuty-Cycle resolution (steps of): %.3f%%\n\tTimerCnt: %lu",
                          frequency_,
                          _frequency,
                          dutyCycleResolution,
                          _timerPeriodTick + 1);
            }

            ASSERT_COND_MSG_ARGS(TO_UNDERLYING(peripheralGroupId_) < SOC_MCPWM_GROUPS,
                                 "MCPWMTimer peripheralGroupId_ must be < SOC_MCPWM_GROUPS(%u)",
                                 SOC_MCPWM_GROUPS);

            mcpwm_timer_config_t timerConfig = {.group_id = TO_UNDERLYING(_groupID),
                                                .clk_src = mcpwm_timer_clock_source_t::MCPWM_TIMER_CLK_SRC_PLL160M,
                                                .resolution_hz = _resolutionHz,
                                                .count_mode = mcpwm_timer_count_mode_t::MCPWM_TIMER_COUNT_MODE_UP,
                                                .period_ticks = _timerPeriodTick,
                                                .intr_priority = DEFAULT_INTERUPT_PRIORITY,
                                                .flags = {.update_period_on_empty = true, .update_period_on_sync = false}};

            esp_err_t retVal = mcpwm_new_timer(&timerConfig, &_timerH);
            ASSERT_COND_MSG_ARGS(retVal == ESP_OK, "mcpwm_new_timer() failed with %u", retVal);

            mcpwm_operator_config_t operatorConfig = {.group_id = TO_UNDERLYING(_groupID),
                                                      .intr_priority = DEFAULT_INTERUPT_PRIORITY,
                                                      .flags{.update_gen_action_on_tez = false,
                                                             .update_gen_action_on_tep = false,
                                                             .update_gen_action_on_sync = false,
                                                             .update_dead_time_on_tez = false,
                                                             .update_dead_time_on_tep = false,
                                                             .update_dead_time_on_sync = false}};
            retVal = mcpwm_new_operator(&operatorConfig, &_operatorH);
            ASSERT_COND_MSG_ARGS(retVal == ESP_OK, "mcpwm_new_operator() failed with %u", retVal);

            retVal = mcpwm_operator_connect_timer(_operatorH, _timerH);
            ASSERT_COND_MSG_ARGS(retVal == ESP_OK, "mcpwm_operator_connect_timer() failed with %u", retVal);
        }

        ~MCPWMTimer()
        {
            ASSERT_MSG("Destructor should never be called at runtime, ressources management is RAII");
        }

      private:
        eMCPWMGroupID getGroupID(void) const
        {
            return _groupID;
        }

        float getFrequency(void) const
        {
            return _frequency;
        }

        uint32_t getResolutionHz(void) const
        {
            return _resolutionHz;
        }

        uint32_t dutyToTickCtn(float duty_) const
        {
            return static_cast<uint32_t>(std::round(static_cast<float>(_timerPeriodTick) * (duty_ / 100.0F)));
        }

        float tickCtnToDuty(uint32_t tickCtn_) const
        {
            return static_cast<float>(tickCtn_) / static_cast<float>(_timerPeriodTick) * 100.0F;
        }

        /**
         * @brief Internal use
         * @attention [WARNING] Can't be called more than SOC_MCPWM_COMPARATORS_PER_OPERATOR per Timer instance
         *
         */
        void createComparator(mcpwm_comparator_config_t& config_, mcpwm_cmpr_handle_t& cmprHandle_)
        {
            esp_err_t retVal = mcpwm_new_comparator(_operatorH, &config_, &cmprHandle_);
            ASSERT_COND_MSG_ARGS(retVal == ESP_OK,
                                 "mcpwm_new_comparator(0x%p, 0x%p, 0x%p) failed with %u",
                                 _operatorH,
                                 &config_,
                                 cmprHandle_,
                                 retVal);
        }

        /**
         * @brief Internal use
         * @attention [WARNING] Can't be called more than SOC_MCPWM_GENERATORS_PER_OPERATOR per Timer instance
         *
         */
        void createGenerator(const mcpwm_generator_config_t& config_, mcpwm_gen_handle_t& genHandle_)
        {
            this->disable();  // Disable possible other channel during this one's init
            esp_err_t retVal = mcpwm_new_generator(_operatorH, &config_, &genHandle_);
            ASSERT_COND_MSG_ARGS(retVal == ESP_OK,
                                 "mcpwm_new_generator(0x%p, 0x%p, 0x%p) failed with %u",
                                 _operatorH,
                                 &config_,
                                 genHandle_,
                                 retVal);
            this->enable();
        }

        void enable(void)
        {
            if (!_enabled)
            {
                _enabled = true;
                esp_err_t retVal = mcpwm_timer_enable(_timerH);
                ASSERT_COND_MSG_ARGS(retVal == ESP_OK, "mcpwm_timer_enable(0x%p) failed with %u", _timerH, retVal);

                retVal = mcpwm_timer_start_stop(_timerH, mcpwm_timer_start_stop_cmd_t::MCPWM_TIMER_START_NO_STOP);
                ASSERT_COND_MSG_ARGS(retVal == ESP_OK, "mcpwm_timer_start_stop(0x%p) failed with %u", _timerH, retVal);
            }
        }

        bool isEnable(void) const
        {
            return _enabled;
        }

        void disable(void)
        {
            if (_enabled)
            {
                _enabled = false;
                esp_err_t retVal = mcpwm_timer_disable(_timerH);
                ASSERT_COND_MSG_ARGS(retVal == ESP_OK, "mcpwm_timer_disable(0x%p) failed with %u", _timerH, retVal);
            }
        }

        static void calculateResTickFromFreq(IN uint32_t frequency_, OUT uint32_t& rResolution_, OUT uint32_t& rTickPeriod_)
        {
            if (frequency_ > MAX_TICK_VALUE)
            {
                rTickPeriod_ = static_cast<uint32_t>(ROUND(static_cast<float>(CLOCK_FREQUENCY_HZ / frequency_) - 1.0F));
                if (rTickPeriod_ == 0)
                {
                    ASSERT_MSG_ARGS("Requested frequency yielded timer period in tick of 0. With selected clock (%u Hz) "
                                    "frequency range is [%u; %u]",
                                    CLOCK_FREQUENCY_HZ,
                                    1U,
                                    CLOCK_FREQUENCY_HZ / 2);

                    rTickPeriod_ = 1;
                }
                rResolution_ = CLOCK_FREQUENCY_HZ / rTickPeriod_ + 1U;
            }
            else if (((CLOCK_FREQUENCY_HZ / frequency_) < MAX_TICK_VALUE))
            {
                rResolution_ = CLOCK_FREQUENCY_HZ;
                rTickPeriod_ = (CLOCK_FREQUENCY_HZ / frequency_) - 1UL;
            }
            else
            {
                rTickPeriod_ = (MAX_TICK_VALUE - MAX_TICK_VALUE % frequency_) - 1UL;
                float resolutionHz = static_cast<float>(rTickPeriod_ + 1UL) * static_cast<float>(frequency_);

                size_t maxIteration = (CLOCK_FREQUENCY_HZ / rTickPeriod_) - 1UL;
                for (size_t i = 0UL; resolutionHz > static_cast<float>(CLOCK_FREQUENCY_HZ) && i < maxIteration; i++)
                {
                    resolutionHz -= static_cast<float>(rTickPeriod_);
                }
                rResolution_ = static_cast<uint32_t>(resolutionHz);
            }
            return;
        }

        float _frequency;
        uint32_t _resolutionHz;
        uint32_t _timerPeriodTick;
        const eMCPWMGroupID _groupID;

        mcpwm_timer_handle_t _timerH = nullptr;
        mcpwm_oper_handle_t _operatorH = nullptr;

        bool _enabled;
    };
}  // namespace PWMGenerators

#endif  // MCPWM_TIMER_HPP
