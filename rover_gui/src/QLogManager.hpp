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
    enum class eLogLevel
    {
        DEBUG,
        INFO,
        WARNING,
        ERROR
    };

    enum class eLogSource
    {
        RTSP_STREAMING,   // RTSP streaming related logs - widget only
        ARUCO_DETECTION,  // Aruco detection related logs - terminal + widget
        SCREENSHOT,       // Screenshot related logs - terminal + widget
        RECORDING,        // Recording related logs - terminal + widget
        GENERAL           // General logs - terminal + widget
    };

    static QLogManager& getInstance(void);

    void log(eLogLevel level, eLogSource source, const QString& message, const QString& target = QString());

    void debug(eLogSource source, const QString& message, const QString& target = QString());
    void info(eLogSource source, const QString& message, const QString& target = QString());
    void warning(eLogSource source, const QString& message, const QString& target = QString());
    void error(eLogSource source, const QString& message, const QString& target = QString());

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

    QString formatLogMessage(eLogLevel level, const QString& message);

    QMap<QString, QSet<eLogLevel>> _enabledLevels;
    QMutex _mutex;
};

namespace LogUtils
{

    constexpr QLogManager::eLogSource RTSP_STREAMING = QLogManager::eLogSource::RTSP_STREAMING;
    constexpr QLogManager::eLogSource ARUCO_DETECTION = QLogManager::eLogSource::ARUCO_DETECTION;
    constexpr QLogManager::eLogSource SCREENSHOT = QLogManager::eLogSource::SCREENSHOT;
    constexpr QLogManager::eLogSource RECORDING = QLogManager::eLogSource::RECORDING;
    constexpr QLogManager::eLogSource GENERAL = QLogManager::eLogSource::GENERAL;

    inline void UI_LOG_DEBUG(QLogManager::eLogSource source, const QString& message, const QString& target = QString())
    {
        QLogManager::getInstance().debug(source, message, target);
    }

    inline void UI_LOG_INFO(QLogManager::eLogSource source, const QString& message, const QString& target = QString())
    {
        QLogManager::getInstance().info(source, message, target);
    }

    inline void UI_LOG_WARNING(QLogManager::eLogSource source, const QString& message, const QString& target = QString())
    {
        QLogManager::getInstance().warning(source, message, target);
    }

    inline void UI_LOG_ERROR(QLogManager::eLogSource source, const QString& message, const QString& target = QString())
    {
        QLogManager::getInstance().error(source, message, target);
    }

    // Specialized logging functions for RTSP
    inline void UI_LOG_DEBUG_RTSP(const QString& message, const QString& target = QString())
    {
        UI_LOG_DEBUG(QLogManager::eLogSource::RTSP_STREAMING, message, target);
    }

    inline void UI_LOG_INFO_RTSP(const QString& message, const QString& target = QString())
    {
        UI_LOG_INFO(QLogManager::eLogSource::RTSP_STREAMING, message, target);
    }

    inline void UI_LOG_WARNING_RTSP(const QString& message, const QString& target = QString())
    {
        UI_LOG_WARNING(QLogManager::eLogSource::RTSP_STREAMING, message, target);
    }

    inline void UI_LOG_ERROR_RTSP(const QString& message, const QString& target = QString())
    {
        UI_LOG_ERROR(QLogManager::eLogSource::RTSP_STREAMING, message, target);
    }

    // Specialized logging functions for ARUCO
    inline void UI_LOG_DEBUG_ARUCO(const QString& message, const QString& target = QString())
    {
        UI_LOG_DEBUG(QLogManager::eLogSource::ARUCO_DETECTION, message, target);
    }

    inline void UI_LOG_INFO_ARUCO(const QString& message, const QString& target = QString())
    {
        UI_LOG_INFO(QLogManager::eLogSource::ARUCO_DETECTION, message, target);
    }

    inline void UI_LOG_WARNING_ARUCO(const QString& message, const QString& target = QString())
    {
        UI_LOG_WARNING(QLogManager::eLogSource::ARUCO_DETECTION, message, target);
    }

    inline void UI_LOG_ERROR_ARUCO(const QString& message, const QString& target = QString())
    {
        UI_LOG_ERROR(QLogManager::eLogSource::ARUCO_DETECTION, message, target);
    }
}  // namespace LogUtils

#endif