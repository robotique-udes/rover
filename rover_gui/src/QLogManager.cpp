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
    QSet<eLogLevel> allLevels;
    allLevels.insert(eLogLevel::DEBUG);
    allLevels.insert(eLogLevel::INFO);
    allLevels.insert(eLogLevel::WARNING);
    allLevels.insert(eLogLevel::ERROR);

    _enabledLevels[""] = allLevels;
}

void QLogManager::log(eLogLevel level, eLogSource source, const QString& message, const QString& target)
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

    QString formattedMessage = this->formatLogMessage(level, message);

    emit newLogMessage(formattedMessage, target);

    if (source != eLogSource::RTSP_STREAMING)
    {
        switch (level)
        {
            case eLogLevel::DEBUG:
                RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "%s", message.toStdString().c_str());
                break;
            case eLogLevel::INFO:
                RCLCPP_INFO(rclcpp::get_logger("GUI"), "%s", message.toStdString().c_str());
                break;
            case eLogLevel::WARNING:
                RCLCPP_WARN(rclcpp::get_logger("GUI"), "%s", message.toStdString().c_str());
                break;
            case eLogLevel::ERROR:
                RCLCPP_ERROR(rclcpp::get_logger("GUI"), "%s", message.toStdString().c_str());
                break;
        }
    }
}

QString QLogManager::formatLogMessage(eLogLevel level, const QString& message)
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz");
    QString levelStr;

    switch (level)
    {
        case eLogLevel::DEBUG:
            levelStr = "DEBUG";
            break;
        case eLogLevel::INFO:
            levelStr = "INFO";
            break;
        case eLogLevel::WARNING:
            levelStr = "WARN";
            break;
        case eLogLevel::ERROR:
            levelStr = "ERROR";
            break;
    }

    return QString("[%1] [%2] %3").arg(timestamp).arg(levelStr).arg(message);
}

void QLogManager::debug(eLogSource source, const QString& message, const QString& target)
{
    this->log(eLogLevel::DEBUG, source, message, target);
}

void QLogManager::info(eLogSource source, const QString& message, const QString& target)
{
    this->log(eLogLevel::INFO, source, message, target);
}

void QLogManager::warning(eLogSource source, const QString& message, const QString& target)
{
    this->log(eLogLevel::WARNING, source, message, target);
}

void QLogManager::error(eLogSource source, const QString& message, const QString& target)
{
    this->log(eLogLevel::ERROR, source, message, target);
}

void QLogManager::setShowDebug(bool show, const QString& target)
{
    QMutexLocker locker(&_mutex);
    QString effectiveTarget = target.isEmpty() ? "" : target;

    if (!_enabledLevels.contains(effectiveTarget))
    {
        _enabledLevels[effectiveTarget] = QSet<eLogLevel>();
        _enabledLevels[effectiveTarget].insert(eLogLevel::INFO);
        _enabledLevels[effectiveTarget].insert(eLogLevel::WARNING);
        _enabledLevels[effectiveTarget].insert(eLogLevel::ERROR);
    }

    if (show)
    {
        _enabledLevels[effectiveTarget].insert(eLogLevel::DEBUG);
    }
    else
    {
        _enabledLevels[effectiveTarget].remove(eLogLevel::DEBUG);
    }
}

void QLogManager::setShowInfo(bool show, const QString& target)
{
    QMutexLocker locker(&_mutex);
    QString effectiveTarget = target.isEmpty() ? "" : target;

    if (!_enabledLevels.contains(effectiveTarget))
    {
        _enabledLevels[effectiveTarget] = QSet<eLogLevel>();
        _enabledLevels[effectiveTarget].insert(eLogLevel::WARNING);
        _enabledLevels[effectiveTarget].insert(eLogLevel::ERROR);
    }

    if (show)
    {
        _enabledLevels[effectiveTarget].insert(eLogLevel::INFO);
    }
    else
    {
        _enabledLevels[effectiveTarget].remove(eLogLevel::INFO);
    }
}

void QLogManager::setShowWarning(bool show, const QString& target)
{
    QMutexLocker locker(&_mutex);
    QString effectiveTarget = target.isEmpty() ? "" : target;

    if (!_enabledLevels.contains(effectiveTarget))
    {
        _enabledLevels[effectiveTarget] = QSet<eLogLevel>();
        _enabledLevels[effectiveTarget].insert(eLogLevel::ERROR);
    }

    if (show)
    {
        _enabledLevels[effectiveTarget].insert(eLogLevel::WARNING);
    }
    else
    {
        _enabledLevels[effectiveTarget].remove(eLogLevel::WARNING);
    }
}

void QLogManager::setShowError(bool show, const QString& target)
{
    QMutexLocker locker(&_mutex);
    QString effectiveTarget = target.isEmpty() ? "" : target;

    if (!_enabledLevels.contains(effectiveTarget))
    {
        _enabledLevels[effectiveTarget] = QSet<eLogLevel>();
    }

    if (show)
    {
        _enabledLevels[effectiveTarget].insert(eLogLevel::ERROR);
    }
    else
    {
        _enabledLevels[effectiveTarget].remove(eLogLevel::ERROR);
    }
}