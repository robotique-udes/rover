#include "QToastNotification.hpp"
#include <QScreen>
#include <qsizepolicy.h>

namespace QHelper
{
    QToastNotification::QToastNotification():
        QWidget(nullptr),
        _shadow(this),
        _fadeInAnim(this, "windowOpacity"),
        _fadeOutAnim(this, "windowOpacity"),
        _slideInAnim(this, "pos"),
        _slideOutAnim(this, "pos"),
        _progressBarAnim(this)
    {
        setupUI();
        setupAnimations();
        setupScreenRect();
    }

    QToastNotification& QToastNotification::getInstance()
    {
        static QToastNotification instance;
        return instance;
    }

    void QToastNotification::setupUI()
    {
        setWindowFlags(Qt::FramelessWindowHint | Qt::ToolTip);
        setAttribute(Qt::WA_TranslucentBackground);
        setAttribute(Qt::WA_ShowWithoutActivating);
        _ui.setupUi(this);

        connect(_ui.closePushButton, &QPushButton::clicked, this, &QToastNotification::hideNotification);

        _progressBarAnim.setTargetObject(_ui.progressBar);
        _progressBarAnim.setPropertyName("value");

        _shadow.setBlurRadius(40);
        _shadow.setOffset(0, 3);
        _shadow.setColor(QColor(0, 0, 0, 220));
        _ui.frame->setGraphicsEffect(&_shadow);

        _ui.frame->setStyleSheet(R"(
            QFrame {
                background-color: #3c3f41;
                border-radius: 8px;
            }
            QFrame:hover {
                background-color: #4d4d4d;
            })");

        _ui.closePushButton->setStyleSheet(R"(
            QPushButton {
                background-color: transparent;
                border-radius: 15px;
                border: none;
                outline: none;
            }
            QPushButton:focus {
                outline: none;
            })");

        _ui.iconSlot->setStyleSheet(R"(
            QPushButton {
                background-color: transparent;
                border-radius: 15px;
                border: none;
                outline: none;
            }
            QPushButton:focus {
                outline: none;
            })");

        _ui.textErrorMessage->setStyleSheet(R"(
            QTextEdit {
                background-color: transparent;
                border-radius: 15px;
                border: none;
                font-size: 16px;
                padding: 5px 10px;
            }
            QTextEdit:focus {
                border: none;
                outline: none;
            })");

        _ui.titleLineEdit->setStyleSheet(R"(
            QLineEdit {
                background-color: transparent;
                border-radius: 15px;
                border: none;
                padding: 5px 10px;
                font-size: 24px;
                font-weight: bold;
            }
            QLineEdit:focus {
                border: none;
                outline: none;
            })");

        _ui.progressBar->setStyleSheet(R"(
            QProgressBar {
                border: none;
                background: rgba(50, 50, 58, 180);
                border-radius: 8px;
                height: 6px;
                margin: 0px;
            }
            QProgressBar::chunk {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00c6ff, stop:1 #0072ff);
                border-radius: 8px;
                margin: 0px;
                min-width: 4px;
            })");
    }

    void QToastNotification::setupAnimations()
    {
        _fadeInAnim.setDuration(300);
        _fadeInAnim.setStartValue(0.0);
        _fadeInAnim.setEndValue(1.0);

        _fadeOutAnim.setDuration(300);
        _fadeOutAnim.setStartValue(1.0);
        _fadeOutAnim.setEndValue(0.0);

        _slideInAnim.setDuration(300);

        _slideOutAnim.setDuration(300);

        _progressBarAnim.setStartValue(100);
        _progressBarAnim.setEndValue(0);

        connect(&_fadeOutAnim, &QPropertyAnimation::finished, this, &QWidget::hide);
        connect(&_slideOutAnim, &QPropertyAnimation::finished, this, &QWidget::hide);

        _closeTimer.setSingleShot(true);
        connect(&_closeTimer, &QTimer::timeout, this, &QToastNotification::hideNotification);
    }

    void QToastNotification::setupScreenRect()
    {
        QList<QScreen*> screens = QGuiApplication::screens();
        QScreen* targetScreen = nullptr;

        if (screens.size() >= 2)
        {
            targetScreen = screens[0];
        }
        else
        {
            targetScreen = QGuiApplication::primaryScreen();
        }

        targetScreenRect = targetScreen->availableGeometry();
    }

    void QToastNotification::notify(const QString& title_, const QString& description_, eNotifType type_, size_t durationMs_)
    {
        _fadeInAnim.stop();
        _fadeOutAnim.stop();
        _slideInAnim.stop();
        _slideOutAnim.stop();
        _progressBarAnim.stop();

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

        _ui.iconSlot->setIcon(icon);
        _ui.iconSlot->setIconSize(QSize(40, 40));
        _ui.textErrorMessage->setText(description_);
        _ui.titleLineEdit->setText(title_);

        this->adjustSize();

        size_t X = targetScreenRect.right() - width() - MARGIN_NOTIF;
        size_t startY = targetScreenRect.bottom() - height() + 2 * MARGIN_NOTIF;
        size_t endY = targetScreenRect.bottom() - height() - 2 * MARGIN_NOTIF;

        _slideInAnim.setStartValue(QPoint(X, startY));
        _slideInAnim.setEndValue(QPoint(X, endY));

        _slideOutAnim.setStartValue(QPoint(X, endY));
        _slideOutAnim.setEndValue(QPoint(X, startY));

        move(X, endY);
        setWindowOpacity(0.0);
        _ui.progressBar->setValue(100);

        this->raise();
        this->show();


        _progressBarAnim.setDuration(durationMs_);

        _fadeInAnim.start();
        _slideInAnim.start();
        _progressBarAnim.start();

        _closeTimer.start(durationMs_);

        QTime currentTime = QTime::currentTime();

        sNotificationInfo data = {currentTime, title_, description_, type_};
        this->saveNotifInfo(data);
    }

    void QToastNotification::hideNotification()
    {
        _fadeInAnim.stop();
        _slideInAnim.stop();
        _progressBarAnim.stop();
        _closeTimer.stop();

        _fadeOutAnim.start();
        _slideOutAnim.start();
    }

    void QToastNotification::saveNotifInfo(const sNotificationInfo& info_)
    {
        history.push_back(info_);

        if (history.size() >= HISTORY_MAX_SIZE)
        {
            history.pop_front();
        }
    }

    QNotificationShowHistory::QNotificationShowHistory():_scrollArea(this),_mainLayout(this)
    {
        _scrollAreaContainer.setLayout(&_scrollAreaLayout);
        _scrollArea.setWidgetResizable(true);

        _scrollArea.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        _scrollAreaContainer.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        //_scrollAreaContainer.setMinimumWidth(_ui.historyScrollArea->width());

        _scrollAreaLayout.setAlignment(Qt::AlignTop);

        _scrollArea.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        _scrollArea.setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

        _mainLayout.addWidget(&_scrollArea);
        setLayout(&_mainLayout);

        _targetScreenRect = QToastNotification::getInstance().targetScreenRect;
        this->hide();
        _scrollAreaContainer.setMinimumSize(400, 400); 
        #warning enlever
        this->setMinimumSize(400, 300);
    }

    void QNotificationShowHistory::showHistory()
    {
        if (_scrollArea.isVisible())
        {
            while (_scrollAreaLayout.count() > 0)
            {
                QLayoutItem* item = _scrollAreaLayout.takeAt(0);
                if (item && item->widget())
                {
                    delete item->widget();
                }
                if (item)
                {
                    delete item;
                }
            }

            _scrollArea.setWidget(nullptr);
            this->hide();
        }

        else
        {
            for (size_t i = 0; i < QToastNotification::getInstance().history.size(); ++i)
            {
                QToastNotification::sNotificationInfo data = QToastNotification::getInstance().history.at(i);
                QNotificationHistoryData* dataWidget
                    = new QNotificationHistoryData(data.timeStamp, data.title, data.description, data.criticityLevel);

                if (dataWidget)
                {
                    _scrollAreaLayout.addWidget(dataWidget);
                }
            }

            _scrollArea.setWidget(&_scrollAreaContainer);
            _scrollAreaContainer.setLayout(&_scrollAreaLayout);
            // Notify layout system
            this->show();
            this->raise();
            this->activateWindow();
        }
    }

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
        _ui.historyIcon->setIconSize(QSize(40, 40));
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
                font-size: 24px;
                font-weight: bold;
            }
            QLineEdit:focus {
                border: none;
                outline: none;
            })");
    }
}  