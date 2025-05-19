#include "QLogManager.hpp"
#include <QDateTime>
#include <QDebug>
#include <rclcpp/rclcpp.hpp>

QLogManager& QLogManager::getInstance(void)
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

void QLogManager::log(eLogLevel level_, eLogSource source_, const QString& message_, const QString& target_)
{
    QMutexLocker locker(&_mutex);

    QString effectiveTarget = target_.isEmpty() ? "" : target_;
    if (!_enabledLevels.contains(effectiveTarget))
    {
        effectiveTarget = "";
    }

    if (!_enabledLevels[effectiveTarget].contains(level_))
    {
        return;
    }

    QString formattedMessage = this->formatLogMessage(level_, message_);

    emit newLogMessage(formattedMessage, target_);

    if (source_ != eLogSource::RTSP_STREAMING)
    {
        switch (level_)
        {
            case eLogLevel::DEBUG:
                RCLCPP_DEBUG(rclcpp::get_logger("GUI"), "%s", message_.toStdString().c_str());
                break;
            case eLogLevel::INFO:
                RCLCPP_INFO(rclcpp::get_logger("GUI"), "%s", message_.toStdString().c_str());
                break;
            case eLogLevel::WARNING:
                RCLCPP_WARN(rclcpp::get_logger("GUI"), "%s", message_.toStdString().c_str());
                break;
            case eLogLevel::ERROR:
                RCLCPP_ERROR(rclcpp::get_logger("GUI"), "%s", message_.toStdString().c_str());
                break;
        }
    }
}

QString QLogManager::formatLogMessage(eLogLevel level_, const QString& message_)
{
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString levelStr;

    switch (level_)
    {
        case eLogLevel::DEBUG:
            levelStr = "DEBUG";
            colorCode = "\033[90m";
            break;
        case eLogLevel::INFO:
            levelStr = "INFO";
            colorCode = "\033[97m";
            break;
        case eLogLevel::WARNING:
            levelStr = "WARN";
            colorCode = "\033[33m";
            break;
        case eLogLevel::ERROR:
            levelStr = "ERROR";
            colorCode = "\033[31m";
            break;
    }

    QString resetCode = "\033[0m";
    
    return QString("%1[%2][%3] %4%5").arg(colorCode).arg(levelStr).arg(timestamp).arg(message_).arg(resetCode);
}

void QLogManager::debug(eLogSource source_, const QString& message_, const QString& target_)
{
    this->log(eLogLevel::DEBUG, source_, message_, target_);
}

void QLogManager::info(eLogSource source_, const QString& message_, const QString& target_)
{
    this->log(eLogLevel::INFO, source_, message_, target_);
}

void QLogManager::warning(eLogSource source_, const QString& message_, const QString& target_)
{
    this->log(eLogLevel::WARNING, source_, message_, target_);
}

void QLogManager::error(eLogSource source_, const QString& message_, const QString& target_)
{
    this->log(eLogLevel::ERROR, source_, message_, target_);
}

void QLogManager::setShowDebug(bool show_, const QString& target_)
{
    QMutexLocker locker(&_mutex);
    QString effectiveTarget = target_.isEmpty() ? "" : target_;

    if (!_enabledLevels.contains(effectiveTarget))
    {
        _enabledLevels[effectiveTarget] = QSet<eLogLevel>();
        _enabledLevels[effectiveTarget].insert(eLogLevel::INFO);
        _enabledLevels[effectiveTarget].insert(eLogLevel::WARNING);
        _enabledLevels[effectiveTarget].insert(eLogLevel::ERROR);
    }

    if (show_)
    {
        _enabledLevels[effectiveTarget].insert(eLogLevel::DEBUG);
    }
    else
    {
        _enabledLevels[effectiveTarget].remove(eLogLevel::DEBUG);
    }
}

void QLogManager::setShowInfo(bool show_, const QString& target_)
{
    QMutexLocker locker(&_mutex);
    QString effectiveTarget = target_.isEmpty() ? "" : target_;

    if (!_enabledLevels.contains(effectiveTarget))
    {
        _enabledLevels[effectiveTarget] = QSet<eLogLevel>();
        _enabledLevels[effectiveTarget].insert(eLogLevel::WARNING);
        _enabledLevels[effectiveTarget].insert(eLogLevel::ERROR);
    }

    if (show_)
    {
        _enabledLevels[effectiveTarget].insert(eLogLevel::INFO);
    }
    else
    {
        _enabledLevels[effectiveTarget].remove(eLogLevel::INFO);
    }
}

void QLogManager::setShowWarning(bool show_, const QString& target_)
{
    QMutexLocker locker(&_mutex);
    QString effectiveTarget = target_.isEmpty() ? "" : target_;

    if (!_enabledLevels.contains(effectiveTarget))
    {
        _enabledLevels[effectiveTarget] = QSet<eLogLevel>();
        _enabledLevels[effectiveTarget].insert(eLogLevel::ERROR);
    }

    if (show_)
    {
        _enabledLevels[effectiveTarget].insert(eLogLevel::WARNING);
    }
    else
    {
        _enabledLevels[effectiveTarget].remove(eLogLevel::WARNING);
    }
}

void QLogManager::setShowError(bool show_, const QString& target_)
{
    QMutexLocker locker(&_mutex);
    QString effectiveTarget = target_.isEmpty() ? "" : target_;

    if (!_enabledLevels.contains(effectiveTarget))
    {
        _enabledLevels[effectiveTarget] = QSet<eLogLevel>();
    }

    if (show_)
    {
        _enabledLevels[effectiveTarget].insert(eLogLevel::ERROR);
    }
    else
    {
        _enabledLevels[effectiveTarget].remove(eLogLevel::ERROR);
    }
}