/********************************************************************************
** Form generated from reading UI file 'Player.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PLAYER_H
#define UI_PLAYER_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RtspPlayerWidget
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *topLayout;
    QLineEdit *rtspUrlInput;
    QPushButton *startButton;
    QPushButton *stopButton;
    QWidget *videoWidget;

    void setupUi(QWidget *RtspPlayerWidget)
    {
        if (RtspPlayerWidget->objectName().isEmpty())
            RtspPlayerWidget->setObjectName(QString::fromUtf8("RtspPlayerWidget"));
        RtspPlayerWidget->resize(800, 600);
        verticalLayout = new QVBoxLayout(RtspPlayerWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        topLayout = new QHBoxLayout();
        topLayout->setObjectName(QString::fromUtf8("topLayout"));
        rtspUrlInput = new QLineEdit(RtspPlayerWidget);
        rtspUrlInput->setObjectName(QString::fromUtf8("rtspUrlInput"));

        topLayout->addWidget(rtspUrlInput);

        startButton = new QPushButton(RtspPlayerWidget);
        startButton->setObjectName(QString::fromUtf8("startButton"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/icons/play.png"), QSize(), QIcon::Normal, QIcon::Off);
        startButton->setIcon(icon);

        topLayout->addWidget(startButton);

        stopButton = new QPushButton(RtspPlayerWidget);
        stopButton->setObjectName(QString::fromUtf8("stopButton"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/icons/stop.png"), QSize(), QIcon::Normal, QIcon::Off);
        stopButton->setIcon(icon1);

        topLayout->addWidget(stopButton);


        verticalLayout->addLayout(topLayout);

        videoWidget = new QWidget(RtspPlayerWidget);
        videoWidget->setObjectName(QString::fromUtf8("videoWidget"));

        verticalLayout->addWidget(videoWidget);


        retranslateUi(RtspPlayerWidget);

        QMetaObject::connectSlotsByName(RtspPlayerWidget);
    } // setupUi

    void retranslateUi(QWidget *RtspPlayerWidget)
    {
        rtspUrlInput->setPlaceholderText(QCoreApplication::translate("RtspPlayerWidget", "Enter RTSP URL...", nullptr));
#if QT_CONFIG(tooltip)
        startButton->setToolTip(QCoreApplication::translate("RtspPlayerWidget", "Start", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        stopButton->setToolTip(QCoreApplication::translate("RtspPlayerWidget", "Pause", nullptr));
#endif // QT_CONFIG(tooltip)
        videoWidget->setStyleSheet(QCoreApplication::translate("RtspPlayerWidget", "background-color: black;", nullptr));
        (void)RtspPlayerWidget;
    } // retranslateUi

};

namespace Ui {
    class RtspPlayerWidget: public Ui_RtspPlayerWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLAYER_H
