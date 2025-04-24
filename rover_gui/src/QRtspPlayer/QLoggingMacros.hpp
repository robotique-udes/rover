#ifndef QT_LOGGING_MACROS_HPP
#define QT_LOGGING_HPP

#include <rclcpp/rclcpp.hpp>
#include <QString>
#include <string>
#include "QLogManager.hpp"

inline std::string to_std_string(const char* str) { return std::string(str); }
inline std::string to_std_string(const std::string& str) { return str; }
inline std::string to_std_string(const QString& str) { return str.toStdString(); }

template<typename T>
inline void log_to_ui(const char* level, const char* component, const T& message, const char* target) {
    QString msgStr = QString::fromStdString(to_std_string(message));
    QLogManager::getInstance().logMessage(QString(level), QString(component), msgStr, QString(target));
}

#define LOG_DEBUG_TARGET(component, message, target) \
    log_to_ui("DEBUG", component, message, target)

#define LOG_INFO_TARGET(component, message, target) \
    log_to_ui("INFO", component, message, target)

#define LOG_WARNING_TARGET(component, message, target) \
    log_to_ui("WARN", component, message, target)

#define LOG_ERROR_TARGET(component, message, target) \
    log_to_ui("ERROR", component, message, target)

#define LOG_DEBUG(component, message) LOG_DEBUG_TARGET(component, message, "global")
#define LOG_INFO(component, message) LOG_INFO_TARGET(component, message, "global")
#define LOG_WARNING(component, message) LOG_WARNING_TARGET(component, message, "global")
#define LOG_ERROR(component, message) LOG_ERROR_TARGET(component, message, "global")

#endif