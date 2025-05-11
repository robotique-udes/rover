#include "QToastNotification.hpp"
#include <QScreen>
#include <qnamespace.h>
#include <qsizepolicy.h>
#include <QApplication>

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

    QRect QToastNotification::getTargetScreenRect(void)
    {
        return _targetScreenRect;
    }

    std::deque<QToastNotification::sNotificationInfo>& QToastNotification::getHistory(void)
    {
        return _history;
    }

    void QToastNotification::setTargetScreenRect(QRect targetScreenRect_)
    {
        _targetScreenRect = targetScreenRect_;
    }
    void QToastNotification::setHistory(std::deque<QToastNotification::sNotificationInfo> history_)
    {
        _history = history_;
    }

    void QToastNotification::notify(const QString& title_, const QString& description_, eNotifType type_, size_t durationMs_)
    {
        _fadeInAnim.stop();
        _fadeOutAnim.stop();
        _closeTimer.stop();
        this->hide();

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
        _ui.iconSlot->setIconSize(QSize(32, 32));
        _ui.textErrorMessage->setText(description_);
        _ui.titleLineEdit->setText(title_);

        this->adjustSize();

        size_t X = this->getTargetScreenRect().right() - width() - MARGIN_NOTIF;
        size_t startY = this->getTargetScreenRect().bottom() - height() + 2 * MARGIN_NOTIF;
        size_t endY = this->getTargetScreenRect().bottom() - height() - 2 * MARGIN_NOTIF;

        _slideInAnim.setStartValue(QPoint(X, startY));
        _slideInAnim.setEndValue(QPoint(X, endY));

        _slideOutAnim.setStartValue(QPoint(X, endY));
        _slideOutAnim.setEndValue(QPoint(X, startY));

        this->move(X, endY);
        this->setWindowOpacity(0.0);
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

    void QToastNotification::QToastNotification::setupUI(void)
    {
        setWindowFlags(Qt::WindowType::FramelessWindowHint | Qt::ToolTip);
        setAttribute(Qt::WidgetAttribute::WA_TranslucentBackground);
        setAttribute(Qt::WidgetAttribute::WA_ShowWithoutActivating);
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

        _ui.closePushButton->setIconSize(QSize(16, 16));
    }

    void QToastNotification::setupAnimations(void)
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

        _closeTimer.setSingleShot(true);
        connect(&_closeTimer, &QTimer::timeout, this, &QToastNotification::hideNotification);
    }

    void QToastNotification::setupScreenRect(void)
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

        this->setTargetScreenRect(targetScreen->availableGeometry());
    }

    void QToastNotification::notifyFromAnyThread(const QString& title_,
                                                 const QString& description_,
                                                 eNotifType type_,
                                                 size_t durationMs_)
    {
        QCoreApplication* pApp = QApplication::instance();
        if (pApp)
        {
            QMetaObject::invokeMethod(
                pApp,
                [this, title_, description_, type_, durationMs_]()
                {
                    notify(title_, description_, type_, durationMs_);
                },
                Qt::QueuedConnection);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "QApplication returned null, something is very wrong");
        }
    }

    void QToastNotification::hideNotification(void)
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
        std::lock_guard<std::mutex> lock(_historyMutex);
        this->getHistory().push_back(info_);

        if (this->getHistory().size() >= HISTORY_MAX_SIZE)
        {
            this->getHistory().pop_front();
        }
    }
}  // namespace QHelper