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
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RtspPlayerWidget
{
public:
    QVBoxLayout *verticalLayout;
    QStackedWidget *mainStackedWidget;
    QWidget *videoPage;
    QVBoxLayout *videoPageLayout;
    QWidget *controlsContainer;
    QVBoxLayout *controlsLayout;
    QHBoxLayout *topLayout;
    QPushButton *arucoButton;
    QLineEdit *arucoIdsTextBox;
    QToolButton *screenshotButton;
    QToolButton *recordButton;
    QSpacerItem *horizontalSpacer;
    QLineEdit *rtspUrlInput;
    QComboBox *streamSelector;
    QPushButton *playPauseButton;
    QFrame *statusIndicator;
    QStackedWidget *videoStack;
    QWidget *videoWidget;
    QWidget *statusPage;
    QVBoxLayout *statusLayout;
    QLabel *statusLabel;
    QHBoxLayout *bottomLayout;
    QPushButton *toggleControlsButton;
    QPushButton *toggleViewButton;
    QSpacerItem *horizontalSpacer_2;
    QWidget *logWidget;
    QVBoxLayout *logLayout;
    QHBoxLayout *logControlLayout;
    QPushButton *backToVideoBtn;
    QSpacerItem *logControlSpacer;
    QCheckBox *debugCheckbox;
    QCheckBox *infoCheckbox;
    QCheckBox *warningCheckbox;
    QCheckBox *errorCheckbox;
    QPushButton *clearButton;
    QTextEdit *logDisplay;

    void setupUi(QWidget *RtspPlayerWidget)
    {
        if (RtspPlayerWidget->objectName().isEmpty())
            RtspPlayerWidget->setObjectName(QString::fromUtf8("RtspPlayerWidget"));
        RtspPlayerWidget->resize(800, 600);
        verticalLayout = new QVBoxLayout(RtspPlayerWidget);
        verticalLayout->setSpacing(0);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        mainStackedWidget = new QStackedWidget(RtspPlayerWidget);
        mainStackedWidget->setObjectName(QString::fromUtf8("mainStackedWidget"));
        videoPage = new QWidget();
        videoPage->setObjectName(QString::fromUtf8("videoPage"));
        videoPageLayout = new QVBoxLayout(videoPage);
        videoPageLayout->setSpacing(0);
        videoPageLayout->setObjectName(QString::fromUtf8("videoPageLayout"));
        videoPageLayout->setContentsMargins(0, 0, 0, 0);
        controlsContainer = new QWidget(videoPage);
        controlsContainer->setObjectName(QString::fromUtf8("controlsContainer"));
        controlsLayout = new QVBoxLayout(controlsContainer);
        controlsLayout->setSpacing(0);
        controlsLayout->setObjectName(QString::fromUtf8("controlsLayout"));
        controlsLayout->setContentsMargins(0, 0, 0, 0);
        topLayout = new QHBoxLayout();
        topLayout->setSpacing(3);
        topLayout->setObjectName(QString::fromUtf8("topLayout"));
        topLayout->setContentsMargins(3, 2, 3, 0);
        arucoButton = new QPushButton(controlsContainer);
        arucoButton->setObjectName(QString::fromUtf8("arucoButton"));
        arucoButton->setMinimumSize(QSize(28, 28));
        arucoButton->setMaximumSize(QSize(28, 28));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/icons/aruco_marker.png"), QSize(), QIcon::Normal, QIcon::Off);
        arucoButton->setIcon(icon);
        arucoButton->setIconSize(QSize(40, 40));
        arucoButton->setCheckable(true);

        topLayout->addWidget(arucoButton);

        arucoIdsTextBox = new QLineEdit(controlsContainer);
        arucoIdsTextBox->setObjectName(QString::fromUtf8("arucoIdsTextBox"));
        arucoIdsTextBox->setMinimumSize(QSize(46, 28));
        arucoIdsTextBox->setMaximumSize(QSize(66, 28));
        arucoIdsTextBox->setAlignment(Qt::AlignCenter);
        arucoIdsTextBox->setReadOnly(true);

        topLayout->addWidget(arucoIdsTextBox);

        screenshotButton = new QToolButton(controlsContainer);
        screenshotButton->setObjectName(QString::fromUtf8("screenshotButton"));
        screenshotButton->setMinimumSize(QSize(28, 28));
        screenshotButton->setMaximumSize(QSize(28, 28));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/icons/camera.png"), QSize(), QIcon::Normal, QIcon::Off);
        screenshotButton->setIcon(icon1);
        screenshotButton->setIconSize(QSize(40, 40));

        topLayout->addWidget(screenshotButton);

        recordButton = new QToolButton(controlsContainer);
        recordButton->setObjectName(QString::fromUtf8("recordButton"));
        recordButton->setMinimumSize(QSize(28, 28));
        recordButton->setMaximumSize(QSize(28, 28));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/icons/record_off.png"), QSize(), QIcon::Normal, QIcon::Off);
        recordButton->setIcon(icon2);
        recordButton->setIconSize(QSize(20, 20));
        recordButton->setCheckable(true);

        topLayout->addWidget(recordButton);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        topLayout->addItem(horizontalSpacer);

        rtspUrlInput = new QLineEdit(controlsContainer);
        rtspUrlInput->setObjectName(QString::fromUtf8("rtspUrlInput"));
        rtspUrlInput->setMinimumSize(QSize(0, 28));
        rtspUrlInput->setMaximumSize(QSize(16777215, 28));

        topLayout->addWidget(rtspUrlInput);

        streamSelector = new QComboBox(controlsContainer);
        streamSelector->setObjectName(QString::fromUtf8("streamSelector"));
        streamSelector->setMinimumSize(QSize(80, 28));
        streamSelector->setMaximumSize(QSize(120, 28));

        topLayout->addWidget(streamSelector);

        playPauseButton = new QPushButton(controlsContainer);
        playPauseButton->setObjectName(QString::fromUtf8("playPauseButton"));
        playPauseButton->setMinimumSize(QSize(28, 28));
        playPauseButton->setMaximumSize(QSize(28, 28));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/icons/play.png"), QSize(), QIcon::Normal, QIcon::Off);
        playPauseButton->setIcon(icon3);
        playPauseButton->setCheckable(true);

        topLayout->addWidget(playPauseButton);

        statusIndicator = new QFrame(controlsContainer);
        statusIndicator->setObjectName(QString::fromUtf8("statusIndicator"));
        statusIndicator->setMinimumSize(QSize(0, 0));
        statusIndicator->setMaximumSize(QSize(0, 0));

        topLayout->addWidget(statusIndicator);


        controlsLayout->addLayout(topLayout);


        videoPageLayout->addWidget(controlsContainer);

        videoStack = new QStackedWidget(videoPage);
        videoStack->setObjectName(QString::fromUtf8("videoStack"));
        videoWidget = new QWidget();
        videoWidget->setObjectName(QString::fromUtf8("videoWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(videoWidget->sizePolicy().hasHeightForWidth());
        videoWidget->setSizePolicy(sizePolicy);
        videoStack->addWidget(videoWidget);
        statusPage = new QWidget();
        statusPage->setObjectName(QString::fromUtf8("statusPage"));
        statusLayout = new QVBoxLayout(statusPage);
        statusLayout->setObjectName(QString::fromUtf8("statusLayout"));
        statusLayout->setAlignment(Qt::AlignCenter);
        statusLabel = new QLabel(statusPage);
        statusLabel->setObjectName(QString::fromUtf8("statusLabel"));
        statusLabel->setAlignment(Qt::AlignCenter);

        statusLayout->addWidget(statusLabel);

        videoStack->addWidget(statusPage);

        videoPageLayout->addWidget(videoStack);

        bottomLayout = new QHBoxLayout();
        bottomLayout->setSpacing(3);
        bottomLayout->setObjectName(QString::fromUtf8("bottomLayout"));
        bottomLayout->setContentsMargins(3, 0, 3, 1);
        toggleControlsButton = new QPushButton(videoPage);
        toggleControlsButton->setObjectName(QString::fromUtf8("toggleControlsButton"));
        toggleControlsButton->setMinimumSize(QSize(28, 28));
        toggleControlsButton->setMaximumSize(QSize(28, 28));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/icons/up_arrow.png"), QSize(), QIcon::Normal, QIcon::Off);
        toggleControlsButton->setIcon(icon4);
        toggleControlsButton->setIconSize(QSize(30, 30));

        bottomLayout->addWidget(toggleControlsButton);

        toggleViewButton = new QPushButton(videoPage);
        toggleViewButton->setObjectName(QString::fromUtf8("toggleViewButton"));
        toggleViewButton->setMinimumSize(QSize(28, 28));
        toggleViewButton->setMaximumSize(QSize(28, 28));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/icons/log_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        toggleViewButton->setIcon(icon5);
        toggleViewButton->setIconSize(QSize(30, 30));

        bottomLayout->addWidget(toggleViewButton);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        bottomLayout->addItem(horizontalSpacer_2);


        videoPageLayout->addLayout(bottomLayout);

        mainStackedWidget->addWidget(videoPage);
        logWidget = new QWidget();
        logWidget->setObjectName(QString::fromUtf8("logWidget"));
        logLayout = new QVBoxLayout(logWidget);
        logLayout->setSpacing(3);
        logLayout->setObjectName(QString::fromUtf8("logLayout"));
        logLayout->setContentsMargins(3, 3, 3, 3);
        logControlLayout = new QHBoxLayout();
        logControlLayout->setSpacing(6);
        logControlLayout->setObjectName(QString::fromUtf8("logControlLayout"));
        logControlLayout->setContentsMargins(0, 0, 0, 3);
        backToVideoBtn = new QPushButton(logWidget);
        backToVideoBtn->setObjectName(QString::fromUtf8("backToVideoBtn"));

        logControlLayout->addWidget(backToVideoBtn);

        logControlSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        logControlLayout->addItem(logControlSpacer);

        debugCheckbox = new QCheckBox(logWidget);
        debugCheckbox->setObjectName(QString::fromUtf8("debugCheckbox"));
        debugCheckbox->setChecked(false);

        logControlLayout->addWidget(debugCheckbox);

        infoCheckbox = new QCheckBox(logWidget);
        infoCheckbox->setObjectName(QString::fromUtf8("infoCheckbox"));
        infoCheckbox->setChecked(true);

        logControlLayout->addWidget(infoCheckbox);

        warningCheckbox = new QCheckBox(logWidget);
        warningCheckbox->setObjectName(QString::fromUtf8("warningCheckbox"));
        warningCheckbox->setChecked(true);

        logControlLayout->addWidget(warningCheckbox);

        errorCheckbox = new QCheckBox(logWidget);
        errorCheckbox->setObjectName(QString::fromUtf8("errorCheckbox"));
        errorCheckbox->setChecked(true);

        logControlLayout->addWidget(errorCheckbox);

        clearButton = new QPushButton(logWidget);
        clearButton->setObjectName(QString::fromUtf8("clearButton"));

        logControlLayout->addWidget(clearButton);


        logLayout->addLayout(logControlLayout);

        logDisplay = new QTextEdit(logWidget);
        logDisplay->setObjectName(QString::fromUtf8("logDisplay"));
        logDisplay->setReadOnly(true);
        logDisplay->setLineWrapMode(QTextEdit::NoWrap);

        logLayout->addWidget(logDisplay);

        mainStackedWidget->addWidget(logWidget);

        verticalLayout->addWidget(mainStackedWidget);


        retranslateUi(RtspPlayerWidget);

        mainStackedWidget->setCurrentIndex(0);
        videoStack->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(RtspPlayerWidget);
    } // setupUi

    void retranslateUi(QWidget *RtspPlayerWidget)
    {
        RtspPlayerWidget->setStyleSheet(QCoreApplication::translate("RtspPlayerWidget", "\n"
"    /* Base styles for all controls */\n"
"    QComboBox, QLineEdit, QPushButton, QToolButton {\n"
"      border: 1px solid #777777;\n"
"      border-radius: 4px;\n"
"      padding: 0px 2px;\n"
"      min-height: 26px;\n"
"      max-height: 26px;\n"
"    }\n"
"    \n"
"    /* Video widget styling */\n"
"    QWidget#videoWidget {\n"
"      background-color: black;\n"
"      border-left: 1px solid #444444;\n"
"      border-right: 1px solid #444444;\n"
"    }\n"
"    \n"
"    /* Control buttons - consistent sizing and reduced padding for larger icons */\n"
"    QPushButton#arucoButton, QPushButton#toggleViewButton, \n"
"    QPushButton#toggleControlsButton, QToolButton#screenshotButton, \n"
"    QToolButton#recordButton, QPushButton#playPauseButton {\n"
"      min-width: 26px;\n"
"      max-width: 26px;\n"
"      min-height: 26px;\n"
"      max-height: 26px;\n"
"      border: 1px solid #777777;\n"
"      border-radius: 4px;\n"
"      padding: 0px;\n"
"    }\n"
"    \n"
"    /* Aruco IDs text box */\n"
"    Q"
                        "LineEdit#arucoIdsTextBox {\n"
"      min-width: 40px;\n"
"      max-width: 60px;\n"
"      color: #e0e0e0;\n"
"      background-color: transparent;\n"
"      border: 1px solid #777777;\n"
"    }\n"
"    \n"
"    /* Style for when IDs are detected */\n"
"    QLineEdit#arucoIdsTextBox[hasIds=\"true\"] {\n"
"      color: white;\n"
"      background-color: rgba(80, 150, 80, 0.3);\n"
"    }\n"
"    \n"
"    /* Status label styling */\n"
"    QLabel#statusLabel {\n"
"      color: white;\n"
"      background-color: rgba(0, 0, 0, 180);\n"
"      padding: 15px;\n"
"      border-radius: 5px;\n"
"      font-weight: bold;\n"
"      font-size: 16px;\n"
"    }\n"
"    \n"
"    /* Log text styling */\n"
"    QTextEdit#logDisplay {\n"
"      background-color: black;\n"
"      color: white; \n"
"      font-family: monospace;\n"
"    }\n"
"    \n"
"    /* Stream header labels - hide them */\n"
"    QLabel.stream-header {\n"
"      max-height: 0px;\n"
"      padding: 0px;\n"
"      margin: 0px;\n"
"      border: none;\n"
"      "
                        "font-size: 0px;\n"
"      color: transparent;\n"
"    }\n"
"   ", nullptr));
#if QT_CONFIG(tooltip)
        arucoButton->setToolTip(QCoreApplication::translate("RtspPlayerWidget", "Enable Aruco marker detection", nullptr));
#endif // QT_CONFIG(tooltip)
        arucoIdsTextBox->setText(QCoreApplication::translate("RtspPlayerWidget", "Ids:", nullptr));
#if QT_CONFIG(tooltip)
        screenshotButton->setToolTip(QCoreApplication::translate("RtspPlayerWidget", "Take Screenshot", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        recordButton->setToolTip(QCoreApplication::translate("RtspPlayerWidget", "Start/stop recording", nullptr));
#endif // QT_CONFIG(tooltip)
        rtspUrlInput->setPlaceholderText(QCoreApplication::translate("RtspPlayerWidget", "Enter RTSP URL...", nullptr));
#if QT_CONFIG(tooltip)
        playPauseButton->setToolTip(QCoreApplication::translate("RtspPlayerWidget", "Play", nullptr));
#endif // QT_CONFIG(tooltip)
        statusIndicator->setStyleSheet(QCoreApplication::translate("RtspPlayerWidget", "QFrame { border-radius: 4px; background-color: red; }", nullptr));
        videoWidget->setStyleSheet(QCoreApplication::translate("RtspPlayerWidget", "background-color: black;", nullptr));
        statusPage->setStyleSheet(QCoreApplication::translate("RtspPlayerWidget", "background-color: black;", nullptr));
        statusLabel->setText(QCoreApplication::translate("RtspPlayerWidget", "Not Connected", nullptr));
#if QT_CONFIG(tooltip)
        toggleControlsButton->setToolTip(QCoreApplication::translate("RtspPlayerWidget", "Hide Controls", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(tooltip)
        toggleViewButton->setToolTip(QCoreApplication::translate("RtspPlayerWidget", "Show Logs", nullptr));
#endif // QT_CONFIG(tooltip)
        backToVideoBtn->setText(QCoreApplication::translate("RtspPlayerWidget", "Back to Video", nullptr));
        debugCheckbox->setText(QCoreApplication::translate("RtspPlayerWidget", "Debug", nullptr));
        infoCheckbox->setText(QCoreApplication::translate("RtspPlayerWidget", "Info", nullptr));
        warningCheckbox->setText(QCoreApplication::translate("RtspPlayerWidget", "Warning", nullptr));
        errorCheckbox->setText(QCoreApplication::translate("RtspPlayerWidget", "Error", nullptr));
        clearButton->setText(QCoreApplication::translate("RtspPlayerWidget", "Clear", nullptr));
    } // retranslateUi

};

namespace Ui {
    class RtspPlayerWidget: public Ui_RtspPlayerWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PLAYER_H
