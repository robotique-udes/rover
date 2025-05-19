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

    void log(eLogLevel level_, eLogSource source_, const QString& message_, const QString& target_ = QString());

    void debug(eLogSource source_, const QString& message_, const QString& target_ = QString());
    void info(eLogSource source_, const QString& message_, const QString& target_ = QString());
    void warning(eLogSource source_, const QString& message_, const QString& target_ = QString());
    void error(eLogSource source_, const QString& message_, const QString& target_ = QString());

    void setShowDebug(bool show_, const QString& target_ = QString());
    void setShowInfo(bool show_, const QString& target_ = QString());
    void setShowWarning(bool show_, const QString& target_ = QString());
    void setShowError(bool show_, const QString& target_ = QString());

  signals:
    void newLogMessage(const QString& message_, const QString& target_);

  private:
    QLogManager();
    ~QLogManager() = default;

    QLogManager(const QLogManager&) = delete;
    QLogManager& operator=(const QLogManager&) = delete;

    QString formatLogMessage(eLogLevel level_, const QString& message_);

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

    template<typename T>
    inline QString toQString(const T& str_)
    {
        if constexpr (std::is_same_v<T, QString>)
        {
            return str_;
        }
        else if constexpr (std::is_same_v<T, std::string>)
        {
            return QString::fromStdString(str_);
        }
        else if constexpr (std::is_same_v<T, const char*>)
        {
            return QString(str_);
        }
        else
        {
            return QString::fromStdString(std::string(str_));
        }
    }

    template<typename StrType, typename TargetType = StrType>
    inline void UI_LOG_DEBUG(QLogManager::eLogSource source_, const StrType& message_, const TargetType& target_ = TargetType())
    {
        QLogManager::getInstance().debug(source_, toQString(message_), toQString(target_));
    }

    template<typename StrType, typename TargetType = StrType>
    inline void UI_LOG_INFO(QLogManager::eLogSource source_, const StrType& message_, const TargetType& target_ = TargetType())
    {
        QLogManager::getInstance().info(source_, toQString(message_), toQString(target_));
    }

    template<typename StrType, typename TargetType = StrType>
    inline void UI_LOG_WARNING(QLogManager::eLogSource source_, const StrType& message_, const TargetType& target_ = TargetType())
    {
        QLogManager::getInstance().warning(source_, toQString(message_), toQString(target_));
    }

    template<typename StrType, typename TargetType = StrType>
    inline void UI_LOG_ERROR(QLogManager::eLogSource source_, const StrType& message_, const TargetType& target_ = TargetType())
    {
        QLogManager::getInstance().error(source_, toQString(message_), toQString(target_));
    }

    template<typename StrType, typename TargetType = StrType>
    inline void UI_LOG_DEBUG_RTSP(const StrType& message_, const TargetType& target_ = TargetType())
    {
        UI_LOG_DEBUG(RTSP_STREAMING, message_, target_);
    }

    template<typename StrType, typename TargetType = StrType>
    inline void UI_LOG_INFO_RTSP(const StrType& message_, const TargetType& target_ = TargetType())
    {
        UI_LOG_INFO(RTSP_STREAMING, message_, target_);
    }

    template<typename StrType, typename TargetType = StrType>
    inline void UI_LOG_WARNING_RTSP(const StrType& message_, const TargetType& target_ = TargetType())
    {
        UI_LOG_WARNING(RTSP_STREAMING, message_, target_);
    }

    template<typename StrType, typename TargetType = StrType>
    inline void UI_LOG_ERROR_RTSP(const StrType& message_, const TargetType& target_ = TargetType())
    {
        UI_LOG_ERROR(RTSP_STREAMING, message_, target_);
    }

    template<typename StrType, typename TargetType = StrType>
    inline void UI_LOG_DEBUG_ARUCO(const StrType& message_, const TargetType& target_ = TargetType())
    {
        UI_LOG_DEBUG(ARUCO_DETECTION, message_, target_);
    }

    template<typename StrType, typename TargetType = StrType>
    inline void UI_LOG_INFO_ARUCO(const StrType& message_, const TargetType& target_ = TargetType())
    {
        UI_LOG_INFO(ARUCO_DETECTION, message_, target_);
    }

    template<typename StrType, typename TargetType = StrType>
    inline void UI_LOG_WARNING_ARUCO(const StrType& message_, const TargetType& target_ = TargetType())
    {
        UI_LOG_WARNING(ARUCO_DETECTION, message_, target_);
    }

    template<typename StrType, typename TargetType = StrType>
    inline void UI_LOG_ERROR_ARUCO(const StrType& message_, const TargetType& target_ = TargetType())
    {
        UI_LOG_ERROR(ARUCO_DETECTION, message_, target_);
    }
}  

#endif