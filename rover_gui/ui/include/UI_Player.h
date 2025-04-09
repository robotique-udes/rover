/********************************************************************************
** Form generated from reading UI file 'Player.ui'
**
** Created by: Qt User Interface Compiler version 6.2.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PLAYER_H
#define UI_PLAYER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
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
    QPushButton *playPauseButton;
    QFrame *statusIndicator;
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

        playPauseButton = new QPushButton(RtspPlayerWidget);
        playPauseButton->setObjectName(QString::fromUtf8("playPauseButton"));
        playPauseButton->setMinimumSize(QSize(40, 40));
        playPauseButton->setMaximumSize(QSize(40, 40));

        topLayout->addWidget(playPauseButton);

        statusIndicator = new QFrame(RtspPlayerWidget);
        statusIndicator->setObjectName(QString::fromUtf8("statusIndicator"));
        statusIndicator->setMinimumSize(QSize(30, 30));
        statusIndicator->setMaximumSize(QSize(30, 30));

        topLayout->addWidget(statusIndicator);


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
        playPauseButton->setToolTip(QCoreApplication::translate("RtspPlayerWidget", "Play/Pause", nullptr));
#endif // QT_CONFIG(tooltip)
        statusIndicator->setStyleSheet(QCoreApplication::translate("RtspPlayerWidget", "\n"
"            QFrame {\n"
"                border-radius: 4px; \n"
"                background-color: red; /* default or when running */\n"
"            }\n"
"            ", nullptr));
        videoWidget->setStyleSheet(QCoreApplication::translate("RtspPlayerWidget", "background-color: black;", nullptr));
        (void)RtspPlayerWidget;
    } // retranslateUi

};

namespace Ui {
    class RtspPlayerWidget: public Ui_RtspPlayerWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLAYER_H
