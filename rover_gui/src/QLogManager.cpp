#include "QLogManager.hpp"
#include <QDateTime>

QLogManager& QLogManager::getInstance()
{
    static QLogManager instance;
    return instance;
}

QLogManager::QLogManager() : QObject(nullptr)
{
    // By default, all log levels are visible for all targets
}

void QLogManager::logMessage(const QString& level, const QString& component, const QString& message, const QString& target)
{
    // Skip if this level is hidden for this target
    if (!isLevelVisible(level, target)) {
        return;
    }
    
    QString formattedMessage = QString("[%1] [%2] [%3] %4")
        .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz"))
        .arg(level)
        .arg(component)
        .arg(message);
    
    emit newLogMessage(formattedMessage, target);
}

void QLogManager::setShowDebug(bool show, const QString& target)
{
    if (show) {
        _hiddenLevels[target].remove("DEBUG");
    } else {
        _hiddenLevels[target].insert("DEBUG");
    }
}

void QLogManager::setShowInfo(bool show, const QString& target)
{
    if (show) {
        _hiddenLevels[target].remove("INFO");
    } else {
        _hiddenLevels[target].insert("INFO");
    }
}

void QLogManager::setShowWarning(bool show, const QString& target)
{
    if (show) {
        _hiddenLevels[target].remove("WARN");
    } else {
        _hiddenLevels[target].insert("WARN");
    }
}

void QLogManager::setShowError(bool show, const QString& target)
{
    if (show) {
        _hiddenLevels[target].remove("ERROR");
    } else {
        _hiddenLevels[target].insert("ERROR");
    }
}

bool QLogManager::isLevelVisible(const QString& level, const QString& target) const
{
    // Check if this target has any hidden levels
    if (!_hiddenLevels.contains(target)) {
        return true;
    }
    
    return !_hiddenLevels[target].contains(level);
}