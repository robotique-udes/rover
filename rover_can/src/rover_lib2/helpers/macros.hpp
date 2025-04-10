#ifndef MACROS_HPP
#define MACROS_HPP

#include <type_traits>

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

#endif  // MACROS_HPP
