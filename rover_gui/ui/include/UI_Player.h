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
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QTextEdit>
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
    QFrame *statusIndicator;
    QStackedWidget *videoStack;
    QWidget *videoWidget;
    QPushButton *toggleViewButton;
    QWidget *statusPage;
    QVBoxLayout *statusLayout;
    QLabel *statusLabel;
    QHBoxLayout *bottomLayout;
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
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(videoPage->sizePolicy().hasHeightForWidth());
        videoPage->setSizePolicy(sizePolicy);
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
        sizePolicy.setHeightForWidth(videoWidget->sizePolicy().hasHeightForWidth());
        videoWidget->setSizePolicy(sizePolicy);
        toggleViewButton = new QPushButton(videoWidget);
        toggleViewButton->setObjectName(QString::fromUtf8("toggleViewButton"));
        toggleViewButton->setGeometry(QRect(770, 560, 28, 28));
        toggleViewButton->setMinimumSize(QSize(28, 28));
        toggleViewButton->setMaximumSize(QSize(28, 28));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/icons/log_view.png"), QSize(), QIcon::Normal, QIcon::Off);
        toggleViewButton->setIcon(icon);
        toggleViewButton->setIconSize(QSize(30, 30));
        videoStack->addWidget(videoWidget);
        statusPage = new QWidget();
        statusPage->setObjectName(QString::fromUtf8("statusPage"));
        statusLayout = new QVBoxLayout(statusPage);
        statusLayout->setObjectName(QString::fromUtf8("statusLayout"));
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
        logDisplay->setLineWrapMode(QTextEdit::NoWrap);
        logDisplay->setReadOnly(true);

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
"    QToolButton#recordButton, QPushButton#playPauseButton,\n"
"    QPushButton#cameraSettingsButton {\n"
"      min-width: 26px;\n"
"      max-width: 26px;\n"
"      min-height: 26px;\n"
"      max-height: 26px;\n"
"      border: 1px solid #777777;\n"
"      border-radius: 4px;\n"
"      padding: 0px;\n"
"    }\n"
"    \n"
""
                        "    /* Aruco IDs text box */\n"
"    QLineEdit#arucoIdsTextBox {\n"
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
"      margin: 0"
                        "px;\n"
"      border: none;\n"
"      font-size: 0px;\n"
"      color: transparent;\n"
"    }\n"
"   ", nullptr));
        statusIndicator->setStyleSheet(QCoreApplication::translate("RtspPlayerWidget", "QFrame { border-radius: 4px; background-color: red; }", nullptr));
        videoWidget->setStyleSheet(QCoreApplication::translate("RtspPlayerWidget", "background-color: black;", nullptr));
#if QT_CONFIG(tooltip)
        toggleViewButton->setToolTip(QCoreApplication::translate("RtspPlayerWidget", "Show Logs", nullptr));
#endif // QT_CONFIG(tooltip)
        statusPage->setStyleSheet(QCoreApplication::translate("RtspPlayerWidget", "background-color: black;", nullptr));
        statusLabel->setText(QCoreApplication::translate("RtspPlayerWidget", "Not Connected", nullptr));
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
