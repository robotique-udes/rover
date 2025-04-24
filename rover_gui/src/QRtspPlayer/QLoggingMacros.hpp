#ifndef QT_LOGGING_MACROS_HPP
#define QT_LOGGING_HPP

#include <rclcpp/rclcpp.hpp>
#include <QString>
#include <string>
#include "QLogManager.hpp"

// Helper functions to convert different types to std::string
inline std::string to_std_string(const char* str) { return std::string(str); }
inline std::string to_std_string(const std::string& str) { return str; }
inline std::string to_std_string(const QString& str) { return str.toStdString(); }

// Helper to format log message for UI log only
template<typename T>
inline void log_to_ui(const char* level, const char* component, const T& message, const char* target) {
    QString msgStr = QString::fromStdString(to_std_string(message));
    QLogManager::getInstance().logMessage(QString(level), QString(component), msgStr, QString(target));
}

// Modified log macros to only send messages to the UI log, not to ROS/console
#define LOG_DEBUG_TARGET(component, message, target) \
    log_to_ui("DEBUG", component, message, target)

#define LOG_INFO_TARGET(component, message, target) \
    log_to_ui("INFO", component, message, target)

#define LOG_WARNING_TARGET(component, message, target) \
    log_to_ui("WARN", component, message, target)

#define LOG_ERROR_TARGET(component, message, target) \
    log_to_ui("ERROR", component, message, target)

// Keep the original macros for backward compatibility, defaulting to "global" target
#define LOG_DEBUG(component, message) LOG_DEBUG_TARGET(component, message, "global")
#define LOG_INFO(component, message) LOG_INFO_TARGET(component, message, "global")
#define LOG_WARNING(component, message) LOG_WARNING_TARGET(component, message, "global")
#define LOG_ERROR(component, message) LOG_ERROR_TARGET(component, message, "global")

// If you need to log to the ROS console as well, use these macros
#define LOG_DEBUG_BOTH(component, message, target) \
    RCLCPP_DEBUG(rclcpp::get_logger(component), "%s", to_std_string(message).c_str()); \
    log_to_ui("DEBUG", component, message, target)

#define LOG_INFO_BOTH(component, message, target) \
    RCLCPP_INFO(rclcpp::get_logger(component), "%s", to_std_string(message).c_str()); \
    log_to_ui("INFO", component, message, target)

#define LOG_WARNING_BOTH(component, message, target) \
    RCLCPP_WARN(rclcpp::get_logger(component), "%s", to_std_string(message).c_str()); \
    log_to_ui("WARN", component, message, target)

#define LOG_ERROR_BOTH(component, message, target) \
    RCLCPP_ERROR(rclcpp::get_logger(component), "%s", to_std_string(message).c_str()); \
    log_to_ui("ERROR", component, message, target)

#endif