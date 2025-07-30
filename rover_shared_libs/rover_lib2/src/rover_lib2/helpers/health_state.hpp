#ifndef ROVER_LIB2_HELPERS_HEALTH_STATE_HPP
#define ROVER_LIB2_HELPERS_HEALTH_STATE_HPP

/**
 * @brief Singleton instance to manage the execution health state of a controller.
 * It must be global because assertions and all canbus managers should be able
 * to modify the health state of the controller. It doesn't make sense to create
 * multiple instances of it because it should be used as a controller wide
 * HealthState; if one part of the code goes into undefined behavior, the whole
 * controller should be considered in undefined behavior.
 *
 * For other applications, a different, not controller wide implementation should
 * be implemented instead.
 */
class HealthState
{
  public:
    /**
     * @brief Returns the singleton HealthState instance or create it if doesn't exist yet
     *
     * @return HealthState&
     */
    static HealthState& getInstance(void)
    {
        static HealthState instance_;
        return instance_;
    }

    /**
     * @brief Set the error flag to true
     *
     */
    void setInError(void)
    {
        _inError = true;
    }

#if defined(GTEST_API_)
    void setInError(bool value_)
    {
        _inError = value_;
    }
#endif  // GTEST_API_

    bool getInError(void)
    {
        return _inError;
    }

  private:
    HealthState() = default;
    HealthState(HealthState const&) = delete;
    void operator=(HealthState const&) = delete;

    bool _inError = false;
};

#endif  // ROVER_LIB2_HELPERS_HEALTH_STATE_HPP
