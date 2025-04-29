#include "QToastNotification.hpp"
#include <QScreen>

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

    void QToastNotification::notify(const std::string& title_,
                                    const std::string& description_,
                                    eNotifType type_,
                                    size_t durationMs_)
    {
        _fadeInAnim.stop();
        _fadeOutAnim.stop();
        _slideInAnim.stop();
        _slideOutAnim.stop();
        _progressBarAnim.stop();

        QIcon icon;

        switch (type_)
        {
            case eNotifType::INFO:
                icon = QApplication::style()->standardIcon(QStyle::SP_MessageBoxInformation);
                break;

            case eNotifType::WARNING:
                icon = QApplication::style()->standardIcon(QStyle::SP_MessageBoxWarning);
                break;

            case eNotifType::ERROR:
                icon = QApplication::style()->standardIcon(QStyle::SP_MessageBoxCritical);
                break;

            default:
                icon = QApplication::style()->standardIcon(QStyle::SP_MessageBoxQuestion);
                break;
        }

        _ui.iconSlot->setIcon(icon);
        _ui.iconSlot->setIconSize(QSize(40, 40));
        _ui.textErrorMessage->setText(QString::fromStdString(description_));
        _ui.titleLineEdit->setText(QString::fromStdString(title_));

        this->adjustSize();

        size_t startX = targetScreenRect.right() + MARGIN_NOTIF;
        size_t endX = targetScreenRect.right() - width() - MARGIN_NOTIF;
        size_t Y = targetScreenRect.bottom() - height() - 2 * MARGIN_NOTIF;

        _slideInAnim.setStartValue(QPoint(startX, Y));
        _slideInAnim.setEndValue(QPoint(endX, Y));

        _slideOutAnim.setStartValue(QPoint(endX, Y));
        _slideOutAnim.setEndValue(QPoint(startX, Y));

        move(startX, Y);
        setWindowOpacity(0.0);
        _ui.progressBar->setValue(100);

        this->show();
        this->raise();

        _progressBarAnim.setDuration(durationMs_);

        _fadeInAnim.start();
        _slideInAnim.start();
        _progressBarAnim.start();

        _closeTimer.start(durationMs_);

        #warning add time stamp here

        sNotificationInfo data = {title_, description_, type_};
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

    QNotificationShowHistory::QNotificationShowHistory()
    {
        setWindowFlags(Qt::FramelessWindowHint | Qt::ToolTip);
        setAttribute(Qt::WA_TranslucentBackground);
        
        _ui_mainWidget.setupUi(this);

        _scrollAreaContainer.setLayout(&_scrollAreaLayout);
        _scrollAreaContainer.setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        _scrollAreaContainer.setMinimumWidth(_ui_mainWidget.historyScrollArea->width());
        
        _scrollAreaLayout.setAlignment(Qt::AlignTop);
        
        this->move(-100, height());

        _ui_mainWidget.historyScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        _ui_mainWidget.historyScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        _ui_mainWidget.historyScrollArea->setVisible(false);

        _targetScreenRect = QToastNotification::getInstance().targetScreenRect;
    }

    void QNotificationShowHistory::showHistory()
    {
        if (_ui_mainWidget.historyScrollArea->isVisible())
        {
            for (size_t i = 0; i < static_cast<size_t>(_scrollAreaLayout.count()); ++i)
            {
                QWidget* widget = _scrollAreaLayout.itemAt(i)->widget();
                if (widget)
                {
                    delete widget;
                }
            }

            _ui_mainWidget.historyScrollArea->setWidget(nullptr);
            _ui_mainWidget.historyScrollArea->setVisible(false);

            size_t offScreenX = _targetScreenRect.right() + 100;
            size_t offScreenY = _targetScreenRect.bottom() + 100;
            this->move(offScreenX, offScreenY);
            this->hide();
        }

        else
        {
            for (size_t i = 0; i < QToastNotification::getInstance().history.size(); ++i)
            {
                QToastNotification::sNotificationInfo data = QToastNotification::getInstance().history.at(i);
                QNotificationHistoryData* dataWidget
                    = new QNotificationHistoryData(data.title, data.description, data.criticityLevel);
                
                if(dataWidget)
                {
                _scrollAreaLayout.addWidget(dataWidget);
                }
            }

            _ui_mainWidget.historyScrollArea->setWidget(&_scrollAreaContainer);
            size_t X = _targetScreenRect.right() - width() - MARGIN;
            size_t Y = _targetScreenRect.bottom() - height() - 8 * MARGIN;

            this->move(X, Y);
            _ui_mainWidget.historyScrollArea->setVisible(true);            
            this->show();
            this->raise();
            this->activateWindow();
        }
    }

    QNotificationHistoryData::QNotificationHistoryData(std::string title_,
                                                       std::string description_,
                                                       QToastNotification::eNotifType type_)
    {
        _ui_subWidget.setupUi(this);
        this->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
        this->setMinimumHeight(170);

        _ui_subWidget.historyTitle->setText(QString::fromStdString(title_));
        _ui_subWidget.historyDescription->setText(QString::fromStdString(description_));

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

            default:
                icon = QApplication::style()->standardIcon(QStyle::SP_MessageBoxQuestion);
                break;
        }

        _ui_subWidget.historyIcon->setIcon(icon);
        _ui_subWidget.historyIcon->setIconSize(QSize(40, 40));
    }

}  // namespace QHelper