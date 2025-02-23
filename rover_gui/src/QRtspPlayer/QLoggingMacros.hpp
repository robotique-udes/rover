#ifndef QT_LOGGING_MACROS_HPP
#define QT_LOGGING_HPP

#include <QDebug>
#include <QString>

#define LOG_DEBUG(component, message) qDebug().noquote() << QString("[%1] %2").arg(component, message)
#define LOG_INFO(component, message) qInfo().noquote() << QString("[%1] %2").arg(component, message)
#define LOG_WARNING(component, message) qWarning().noquote() << QString("[%1] %2").arg(component, message)
#define LOG_ERROR(component, message) qCritical().noquote() << QString("[%1] %2").arg(component, message)

#endif 