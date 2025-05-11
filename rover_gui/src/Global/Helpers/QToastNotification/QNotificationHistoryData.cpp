#include "QNotificationHistoryData.hpp"

#include <QScreen>
#include <qnamespace.h>
#include <qsizepolicy.h>

namespace QHelper
{
    
    QNotificationHistoryData::QNotificationHistoryData(QTime timeStamp_,
                                                       QString title_,
                                                       QString description_,
                                                       QToastNotification::eNotifType type_)
    {
        _ui.setupUi(this);
        this->setStyle();
        this->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);

        _ui.historyTitle->setText(title_);
        _ui.historyDescription->setText(description_);
        _ui.timestamp->setText(timeStamp_.toString("HH:mm:ss"));

        QIcon icon;

        switch (type_)
        {
            case QToastNotification::eNotifType::INFO:
                icon = QApplication::style()->standardIcon(QStyle::SP_MessageBoxInformation);
                break;

            case QToastNotification::eNotifType::WARNING:
                icon = QApplication::style()->standardIcon(QStyle::SP_MessageBoxWarning);
                break;

            case QToastNotification::eNotifType::ERROR:
                icon = QApplication::style()->standardIcon(QStyle::SP_MessageBoxCritical);
                break;

            case QToastNotification::eNotifType::SUCCESS:
                icon = QApplication::style()->standardIcon(QStyle::SP_DialogApplyButton);
                break;

            default:
                icon = QApplication::style()->standardIcon(QStyle::SP_MessageBoxQuestion);
                break;
        }

        _ui.historyIcon->setIcon(icon);
        _ui.historyIcon->setIconSize(QSize(32, 32));
    }

    void QNotificationHistoryData::setStyle()
    {
        _ui.timestamp->setStyleSheet(R"(
            QLineEdit {
                background-color: transparent;
                border-radius: 15px;
                border: none;
                padding: 5px 10px;
                font-size: 14px;
                font-weight: bold;
            }
            QLineEdit:focus {
                border: none;
                outline: none;
            })");

        _ui.historyFrame->setStyleSheet(R"(
            QFrame {
                background-color: #3c3f41;
                border-radius: 8px;
            }
            QFrame:hover {
                background-color: #4d4d4d;
            })");

        _ui.historyIcon->setStyleSheet(R"(
            QPushButton {
                background-color: transparent;
                border-radius: 15px;
                border: none;
                outline: none;
            }
            QPushButton:focus {
                outline: none;
            })");

        _ui.historyDescription->setStyleSheet(R"(
            QTextEdit {
                background-color: transparent;
                border-radius: 15px;
                border: none;
                font-size: 16px;
                padding: 5px 10px;
                font-weight: bold;
            }
            QTextEdit:focus {
                border: none;
                outline: none;
            })");

        _ui.historyTitle->setStyleSheet(R"(
            QLineEdit {
                background-color: transparent;
                border-radius: 15px;
                border: none;
                padding: 5px 10px;
                font-size: 20px;
                font-weight: bold;
            }
            QLineEdit:focus {
                border: none;
                outline: none;
            })");
    }
}  // namespace QHelper