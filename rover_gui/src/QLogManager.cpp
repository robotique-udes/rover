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

    _enabledLevels[0] = allLevels;
}

void QLogManager::log(eLogLevel level_, eLogSource source_, const QString& message_, QWidget* target_)
{
    QMutexLocker locker(&_mutex);

    quintptr key = reinterpret_cast<quintptr>(target_);
    if (!_enabledLevels.contains(key))
    {
        key = 0;
    }

    if (!_enabledLevels[key].contains(level_))
    {
        return;
    }

    QString formattedMessage = this->formatLogMessageHtml(level_, message_);

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

QString QLogManager::formatLogMessageHtml(eLogLevel level_, const QString& message_)
{
    QString timestamp = QDateTime::currentDateTime().toString("mm:ss:ms");
    QString levelStr;
    QString levelClass;

    switch (level_)
    {
        case eLogLevel::DEBUG:
            levelStr = "DEBUG";
            levelClass = "debug";
            break;
        case eLogLevel::INFO:
            levelStr = "INFO";
            levelClass = "info";
            break;
        case eLogLevel::WARNING:
            levelStr = "WARN";
            levelClass = "warning";
            break;
        case eLogLevel::ERROR:
            levelStr = "ERROR";
            levelClass = "error";
            break;
    }

    return QString("<span class='%1'>[%2][%3] %4</span>")
        .arg(levelClass)
        .arg(levelStr)
        .arg(timestamp)
        .arg(message_.toHtmlEscaped());
}

void QLogManager::debug(eLogSource source_, const QString& message_, QWidget* target_)
{
    this->log(eLogLevel::DEBUG, source_, message_, target_);
}

void QLogManager::info(eLogSource source_, const QString& message_, QWidget* target_)
{
    this->log(eLogLevel::INFO, source_, message_, target_);
}

void QLogManager::warning(eLogSource source_, const QString& message_, QWidget* target_)
{
    this->log(eLogLevel::WARNING, source_, message_, target_);
}

void QLogManager::error(eLogSource source_, const QString& message_, QWidget* target_)
{
    this->log(eLogLevel::ERROR, source_, message_, target_);
}

void QLogManager::setShowDebug(bool show_, QWidget* target_)
{
    QMutexLocker locker(&_mutex);
    quintptr key = reinterpret_cast<quintptr>(target_);

    if (!_enabledLevels.contains(key))
    {
        _enabledLevels[key] = QSet<eLogLevel>();
        _enabledLevels[key].insert(eLogLevel::INFO);
        _enabledLevels[key].insert(eLogLevel::WARNING);
        _enabledLevels[key].insert(eLogLevel::ERROR);
    }

    if (show_)
    {
        _enabledLevels[key].insert(eLogLevel::DEBUG);
    }
    else
    {
        _enabledLevels[key].remove(eLogLevel::DEBUG);
    }
}

void QLogManager::setShowInfo(bool show_, QWidget* target_)
{
    QMutexLocker locker(&_mutex);
    quintptr key = reinterpret_cast<quintptr>(target_);

    if (!_enabledLevels.contains(key))
    {
        _enabledLevels[key] = QSet<eLogLevel>();
        _enabledLevels[key].insert(eLogLevel::WARNING);
        _enabledLevels[key].insert(eLogLevel::ERROR);
    }

    if (show_)
    {
        _enabledLevels[key].insert(eLogLevel::INFO);
    }
    else
    {
        _enabledLevels[key].remove(eLogLevel::INFO);
    }
}

void QLogManager::setShowWarning(bool show_, QWidget* target_)
{
    QMutexLocker locker(&_mutex);
    quintptr key = reinterpret_cast<quintptr>(target_);

    if (!_enabledLevels.contains(key))
    {
        _enabledLevels[key] = QSet<eLogLevel>();
        _enabledLevels[key].insert(eLogLevel::ERROR);
    }

    if (show_)
    {
        _enabledLevels[key].insert(eLogLevel::WARNING);
    }
    else
    {
        _enabledLevels[key].remove(eLogLevel::WARNING);
    }
}

void QLogManager::setShowError(bool show_, QWidget* target_)
{
    QMutexLocker locker(&_mutex);
    quintptr key = reinterpret_cast<quintptr>(target_);

    if (!_enabledLevels.contains(key))
    {
        _enabledLevels[key] = QSet<eLogLevel>();
    }

    if (show_)
    {
        _enabledLevels[key].insert(eLogLevel::ERROR);
    }
    else
    {
        _enabledLevels[key].remove(eLogLevel::ERROR);
    }
}