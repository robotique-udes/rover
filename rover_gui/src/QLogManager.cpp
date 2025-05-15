#include "QLogManager.hpp"
#include <QDateTime>
#include <QDebug>
#include <rclcpp/rclcpp.hpp>

QLogManager& QLogManager::getInstance()
{
    static QLogManager instance;
    return instance;
}

QLogManager::QLogManager():
    QObject(nullptr)
{
    QSet<LogLevel> allLevels;
    allLevels.insert(DEBUG);
    allLevels.insert(INFO);
    allLevels.insert(WARNING);
    allLevels.insert(ERROR);

    _enabledLevels[""] = allLevels;
}

void QLogManager::log(LogLevel level, LogSource source, const QString& message, const QString& target)
{
    QMutexLocker locker(&_mutex);

    QString effectiveTarget = target.isEmpty() ? "" : target;
    if (!_enabledLevels.contains(effectiveTarget))
    {
        effectiveTarget = "";
    }

    if (!_enabledLevels[effectiveTarget].contains(level))
    {
        return;
    }

    QString formattedMessage = formatLogMessage(level, message);

    emit newLogMessage(formattedMessage, target);

    if (source != RTSP_STREAMING)
    {
        switch (level)
        {
            case DEBUG:
                RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "%s", message.toStdString().c_str());
                break;
            case INFO:
                RCLCPP_INFO(rclcpp::get_logger("GUI"), "%s", message.toStdString().c_str());
                break;
            case WARNING:
                RCLCPP_WARN(rclcpp::get_logger("GUI"), "%s", message.toStdString().c_str());
                break;
            case ERROR:
                RCLCPP_ERROR(rclcpp::get_logger("GUI"), "%s", message.toStdString().c_str());
                break;
        }
    }
}

QString QLogManager::formatLogMessage(LogLevel level, const QString& message)
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
    QString levelStr;

    switch (level)
    {
        case DEBUG:
            levelStr = "DEBUG";
            break;
        case INFO:
            levelStr = "INFO";
            break;
        case WARNING:
            levelStr = "WARN";
            break;
        case ERROR:
            levelStr = "ERROR";
            break;
    }

    return QString("[%1] [%2] %3").arg(timestamp).arg(levelStr).arg(message);
}

void QLogManager::debug(LogSource source, const QString& message, const QString& target)
{
    log(DEBUG, source, message, target);
}

void QLogManager::info(LogSource source, const QString& message, const QString& target)
{
    log(INFO, source, message, target);
}

void QLogManager::warning(LogSource source, const QString& message, const QString& target)
{
    log(WARNING, source, message, target);
}

void QLogManager::error(LogSource source, const QString& message, const QString& target)
{
    log(ERROR, source, message, target);
}

void QLogManager::setShowDebug(bool show, const QString& target)
{
    QMutexLocker locker(&_mutex);
    QString effectiveTarget = target.isEmpty() ? "" : target;

    if (!_enabledLevels.contains(effectiveTarget))
    {
        _enabledLevels[effectiveTarget] = QSet<LogLevel>();
        _enabledLevels[effectiveTarget].insert(INFO);
        _enabledLevels[effectiveTarget].insert(WARNING);
        _enabledLevels[effectiveTarget].insert(ERROR);
    }

    if (show)
    {
        _enabledLevels[effectiveTarget].insert(DEBUG);
    }
    else
    {
        _enabledLevels[effectiveTarget].remove(DEBUG);
    }
}

void QLogManager::setShowInfo(bool show, const QString& target)
{
    QMutexLocker locker(&_mutex);
    QString effectiveTarget = target.isEmpty() ? "" : target;

    if (!_enabledLevels.contains(effectiveTarget))
    {
        _enabledLevels[effectiveTarget] = QSet<LogLevel>();
        _enabledLevels[effectiveTarget].insert(WARNING);
        _enabledLevels[effectiveTarget].insert(ERROR);
    }

    if (show)
    {
        _enabledLevels[effectiveTarget].insert(INFO);
    }
    else
    {
        _enabledLevels[effectiveTarget].remove(INFO);
    }
}

void QLogManager::setShowWarning(bool show, const QString& target)
{
    QMutexLocker locker(&_mutex);
    QString effectiveTarget = target.isEmpty() ? "" : target;

    if (!_enabledLevels.contains(effectiveTarget))
    {
        _enabledLevels[effectiveTarget] = QSet<LogLevel>();
        _enabledLevels[effectiveTarget].insert(ERROR);
    }

    if (show)
    {
        _enabledLevels[effectiveTarget].insert(WARNING);
    }
    else
    {
        _enabledLevels[effectiveTarget].remove(WARNING);
    }
}

void QLogManager::setShowError(bool show, const QString& target)
{
    QMutexLocker locker(&_mutex);
    QString effectiveTarget = target.isEmpty() ? "" : target;

    if (!_enabledLevels.contains(effectiveTarget))
    {
        _enabledLevels[effectiveTarget] = QSet<LogLevel>();
    }

    if (show)
    {
        _enabledLevels[effectiveTarget].insert(ERROR);
    }
    else
    {
        _enabledLevels[effectiveTarget].remove(ERROR);
    }
}