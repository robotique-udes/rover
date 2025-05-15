#ifndef MACROS_HPP
#define MACROS_HPP

#include <cmath>
#include <type_traits>

#if defined(__linux__) && defined(RCLCPP_DEBUG)
#include <ament_index_cpp/get_package_prefix.hpp>
#endif  // defined(__linux__) && defined(RCLCPP_DEBUG)

template<typename ENUM_T>
constexpr std::underlying_type_t<ENUM_T> TO_UNDERLYING(ENUM_T e) noexcept
{
    static_assert(std::is_enum_v<ENUM_T>, "TO_UNDERLYING() can only be used with enum types");
    return static_cast<std::underlying_type_t<ENUM_T>>(e);
}

#if !defined(ARDUINO_ESP32S3_DEV)
#define __FILENAME__ (__builtin_strrchr("/" __FILE__, '/') + 1)
#endif  // !defined(ARDUINO_ESP32S3_DEV)

#define EVER \
    ;        \
    ;

#define IN
#define OUT
#define INOUT

// Necessary for following macros because VSCode's Microsoft CPP language server doesn't work with template and throws a bunch of
// false positive errors
#ifdef __INTELLISENSE__
#pragma diag_suppress 1919  // Parameter pack expension
#endif

/**
 * @brief Checks if types derive from a base
 * @note Usage example: VALIDATE_TYPE(SubscriberBaseT (base type), SubT (derived class, can be parameter pack))
 *
 */
#define VALIDATE_BASE_TYPE(BaseT, DerivedT) \
    static_assert((std::is_base_of_v<BaseT, DerivedT>), "Template argument " #DerivedT " must be derived from " #BaseT)

/**
 * @brief Checks if types derive from a base
 * @note Usage example: VALIDATE_TYPE(SubscriberBaseT (base type), SubT (derived class, can be parameter pack))
 *
 */
#define VALIDATE_BASE_TYPE_PACK(BaseT, ...)                                                \
    static_assert((... && std::is_base_of_v<BaseT, std::remove_reference_t<__VA_ARGS__>>), \
                  "All template arguments must be derived from " #BaseT)

template<typename T>
constexpr T ABS(T var_) noexcept
{
    if constexpr (std::is_unsigned_v<T>)
    {
        return var_;
    }
    else
    {
        return ((var_ < static_cast<T>(0)) ? -var_ : var_);
    }
}

template<typename T>
constexpr bool IN_ERROR(T var_, T error_, T goal_)
{
    error_ = ABS(error_);
    return var_ <= (goal_ + error_) && var_ >= (goal_ - error_);
}

template<typename T>
constexpr T CONSTRAIN(T value_, T min_, T max_)
{
    if (value_ < min_)
    {
        return min_;
    }
    else if (value_ > max_)
    {
        return max_;
    }
    else
    {
        return value_;
    }
}

/**
 * @brief Truncate a floating-point value (remove fractional part).
 */
template<std::floating_point T>
constexpr T TRUNC(T value_)
{
    return (value_ == 0) ? value_ :  // Handle ±0.0
               (value_ > 0) ? static_cast<T>(static_cast<int64_t>(value_))
                            : static_cast<T>(static_cast<int64_t>(value_ - static_cast<T>(1.0)) + static_cast<T>(1.0));
}

/**
 * @brief Round a floating-point value to the nearest integer
 */
template<std::floating_point T>
constexpr T ROUND(T value_)
{
    T absValue = ABS(value_);
    T fractionalPart = absValue - static_cast<int64_t>(absValue);

    if (fractionalPart >= static_cast<T>(0.5))
    {
        return (value_ < 0) ? static_cast<T>(static_cast<int64_t>(value_) - 1) : static_cast<T>(static_cast<int64_t>(value_) + 1);
    }
    return static_cast<T>(static_cast<int64_t>(value_));
}

/**
 * @brief Round a value up to the next integer (eq of ceil)
 */
template<std::floating_point T>
constexpr T ROUND_UP(T value_)
{
    T integerPart = static_cast<T>(static_cast<int64_t>(value_));
    if (value_ == integerPart)
    {
        return integerPart;
    }
    return (value_ > 0) ? integerPart + static_cast<T>(1) : integerPart;
}

/**
 * @brief Round a value down to the previous integer (eq of floor)
 */
template<std::floating_point T>
constexpr T ROUND_DOWN(T value_)
{
    T integerPart = static_cast<T>(static_cast<int64_t>(value_));
    if (value_ == integerPart)
    {
        return integerPart;
    }
    return (value_ < 0) ? integerPart - static_cast<T>(1) : integerPart;
}

#define MAP(x, in_min, in_max, out_min, out_max)                                                                  \
    (((float)(x) - (float)(in_min)) * ((float)(out_max) - (float)(out_min)) / ((float)(in_max) - (float)(in_min)) \
     + (float)(out_min))

#define CHECK_POINTER_VALID(POINTER) (POINTER ? true : false)

#define SIGN(VAR) ((float)VAR > 0.0f ? 1.0f : -1.0f)

#define GET_WORSE_OF(A, B) (A == true && B == true)

// Removes unused argument warning
#define REMOVE_UNUSED(x) (void)(x)

#if defined(__linux__) && defined(RCLCPP_DEBUG)
#define GET_PACKAGE_SOURCE_DIR(package_name) \
    (ament_index_cpp::get_package_prefix(package_name) + "/../../src/rover/" + package_name)
#endif  // defined(_linux_) && defined(RCLCPP_DEBUG

#endif  // MACROS_HPP
