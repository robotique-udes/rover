#ifndef ROVER_LIB2_HELPERS_THREAD_SAFE_ACCESS_HPP
#define ROVER_LIB2_HELPERS_THREAD_SAFE_ACCESS_HPP

#if defined(__linux__)

#include <mutex>

/**
 * @brief Thread safe access to a protected variable, the variable access will be locked until the end of this object's
 * lifetime. Prefer copying the data if write access isn't needed to keeping locking time short.
 *
 */
template<typename T>
class ThreadSafeAccess
{
  public:
    ThreadSafeAccess(std::mutex& mtx_, T& data_):
        _lock(mtx_),
        _data(data_)
    {
    }

    T& getThreadSafeAccess(void)
    {
        return _data;
    }

  private:
    std::lock_guard<std::mutex> _lock;
    T& _data;
};

#endif  // defined(__linux__)

#endif  // ROVER_LIB2_HELPERS_THREAD_SAFE_ACCESS_HPP
