#ifndef QLOGMANAGER_HPP
#define QLOGMANAGER_HPP

#include <QObject>
#include <QString>
#include <QMap>
#include <QSet>
#include <QMutex>

class QLogManager : public QObject
{
    Q_OBJECT

  public:
    enum LogLevel
    {
        DEBUG,
        INFO,
        WARNING,
        ERROR
    };

    enum LogSource
    {
        RTSP_STREAMING,   // RTSP streaming related logs - widget only
        ARUCO_DETECTION,  // Aruco detection related logs - terminal + widget
        SCREENSHOT,       // Screenshot related logs - terminal + widget
        RECORDING,        // Recording related logs - terminal + widget
        GENERAL           // General logs - terminal + widget
    };

    static QLogManager& getInstance();

    void log(LogLevel level, LogSource source, const QString& message, const QString& target = QString());

    void debug(LogSource source, const QString& message, const QString& target = QString());
    void info(LogSource source, const QString& message, const QString& target = QString());
    void warning(LogSource source, const QString& message, const QString& target = QString());
    void error(LogSource source, const QString& message, const QString& target = QString());

    void setShowDebug(bool show, const QString& target = QString());
    void setShowInfo(bool show, const QString& target = QString());
    void setShowWarning(bool show, const QString& target = QString());
    void setShowError(bool show, const QString& target = QString());

  signals:

    void newLogMessage(const QString& message, const QString& target);

  private:
    QLogManager();
    ~QLogManager() = default;

    QLogManager(const QLogManager&) = delete;
    QLogManager& operator=(const QLogManager&) = delete;

    QString formatLogMessage(LogLevel level, const QString& message);

    QMap<QString, QSet<LogLevel>> _enabledLevels;
    QMutex _mutex;
};

#define UI_LOG_DEBUG(source, message, target) QLogManager::getInstance().debug(QLogManager::source, message, target)
#define UI_LOG_INFO(source, message, target) QLogManager::getInstance().info(QLogManager::source, message, target)
#define UI_LOG_WARNING(source, message, target) QLogManager::getInstance().warning(QLogManager::source, message, target)
#define UI_LOG_ERROR(source, message, target) QLogManager::getInstance().error(QLogManager::source, message, target)

#define UI_LOG_DEBUG_RTSP(message, target) UI_LOG_DEBUG(RTSP_STREAMING, message, target)
#define UI_LOG_INFO_RTSP(message, target) UI_LOG_INFO(RTSP_STREAMING, message, target)
#define UI_LOG_WARNING_RTSP(message, target) UI_LOG_WARNING(RTSP_STREAMING, message, target)
#define UI_LOG_ERROR_RTSP(message, target) UI_LOG_ERROR(RTSP_STREAMING, message, target)

#define UI_LOG_DEBUG_ARUCO(message, target) UI_LOG_DEBUG(ARUCO_DETECTION, message, target)
#define UI_LOG_INFO_ARUCO(message, target) UI_LOG_INFO(ARUCO_DETECTION, message, target)
#define UI_LOG_WARNING_ARUCO(message, target) UI_LOG_WARNING(ARUCO_DETECTION, message, target)
#define UI_LOG_ERROR_ARUCO(message, target) UI_LOG_ERROR(ARUCO_DETECTION, message, target)

#endif