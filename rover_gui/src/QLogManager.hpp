#ifndef Q_LOG_MANAGER_HPP
#define Q_LOG_MANAGER_HPP

#include <QString>
#include <QObject>
#include <QSet>
#include <QMap>

enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR
};

class QLogManager : public QObject
{
    Q_OBJECT
    
public:
    static QLogManager& getInstance();
    
    void logMessage(const QString& level, const QString& component, const QString& message, const QString& target = "global");
    
    void setShowDebug(bool show, const QString& target = "global");
    void setShowInfo(bool show, const QString& target = "global");
    void setShowWarning(bool show, const QString& target = "global");
    void setShowError(bool show, const QString& target = "global");
    
    bool isLevelVisible(const QString& level, const QString& target = "global") const;
    
signals:
    void newLogMessage(const QString& formattedMessage, const QString& target);
    
private:
    QLogManager();
    ~QLogManager() = default;
    
    QLogManager(const QLogManager&) = delete;
    QLogManager& operator=(const QLogManager&) = delete;
    
    QMap<QString, QSet<QString>> _hiddenLevels; 
};

#endif 