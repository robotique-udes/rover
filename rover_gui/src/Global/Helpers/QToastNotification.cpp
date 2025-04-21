#include "QToastNotification.hpp"
#include <QScreen>

QToastNotification::QToastNotification():
    QWidget(nullptr),
    _shadow(this),
    _fadeInAnim(this,"windowOpacity"),
    _fadeOutAnim(this,"windowOpacity"),
    _slideInAnim(this,"pos"),
    _slideOutAnim(this,"pos"),
    _progressBarAnim()
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
    _shadow.setBlurRadius(10);
    _shadow.setOffset(0, 3);
    _shadow.setColor(QColor(0, 0, 0, 160));
    _ui.frame->setGraphicsEffect(&_shadow);
    
    _ui.frame->setStyleSheet("background-color: #4a4e54;"
                             "border-radius: 15px;"
                             "border: 1px solid #6c7075;");
    _ui.closePushButton->setStyleSheet("QPushButton {"
                                       "    background-color: #4a4e54;"
                                       "    border-radius: 15px;"
                                       "    border: none;"
                                       "    outline: none;"
                                       "}"
                                       "QPushButton:focus {"
                                       "    outline: none;"
                                       "}");

    _ui.iconSlot->setStyleSheet("QPushButton {"
                                "    background-color: #4a4e54;"
                                "    border-radius: 15px;"
                                "    border: none;"
                                "    outline: none;"
                                "}"
                                "QPushButton:focus {"
                                "    outline: none;"
                                "}");
    _ui.textErrorMessage->setStyleSheet("QTextEdit {"
                                        "    background-color: #4a4e54;"
                                        "    border-radius: 15px;"
                                        "    border: none;"
                                        "    font-size: 16px;"
                                        "    padding: 5px 10px;"
                                        "}"
                                        "QTextEdit:focus {"
                                        "    border: none;"
                                        "    outline: none;"
                                        "}");
    _ui.titleLineEdit->setStyleSheet("QLineEdit {"
                                     "    background-color: #4a4e54;"
                                     "    border-radius: 15px;"
                                     "    border: none;"
                                     "    padding: 5px 10px;"
                                     "    font-size: 24px;"
                                     "    font-weight: bold;"
                                     "}"
                                     "QLineEdit:focus {"
                                     "    border: none;"
                                     "    outline: none;"
                                     "}");

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
                                        min-width: 4px; /* <- important! ensures chunk stays visible */
                                    }
                                    )");
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
    connect(&_closeTimer,
            &QTimer::timeout,
            this,
            &QToastNotification::hideNotification);
}

void QToastNotification::setupScreenRect()
{
    QList<QScreen*> screens = QGuiApplication::screens();
    QScreen* targetScreen = nullptr;

    if (screens.size() >= 2)
    {
        targetScreen = screens[1];
    }
    else
    {
        targetScreen = QGuiApplication::primaryScreen();
    }

    _targetScreenRect = targetScreen->availableGeometry();
}

void QToastNotification::notify(const QString& title_, const QString& description_, eNotifType type_, int durationMs_)
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
            icon = QApplication::style()->standardIcon(QStyle::SP_MessageBoxQuestion);  // Optional fallback
            break;
    }

    _ui.iconSlot->setIcon(icon);
    _ui.iconSlot->setIconSize(QSize(40, 40));
    _ui.textErrorMessage->setText(description_);
    _ui.titleLineEdit->setText(title_);

    adjustSize();

    const int margin = 20;
    int startX = _targetScreenRect.right() + margin;
    int endX = _targetScreenRect.right() - width() - margin;
    int yPos = _targetScreenRect.bottom() - height() - 2*margin;



    _slideInAnim.setStartValue(QPoint(startX, yPos));
    _slideInAnim.setEndValue(QPoint(endX, yPos));

    _slideOutAnim.setStartValue(QPoint(endX, yPos));
    _slideOutAnim.setEndValue(QPoint(startX, yPos));

    _progressBarAnim.setDuration(durationMs_);

    move(startX, yPos);
    setWindowOpacity(0.0);
    _ui.progressBar->setValue(100);

    show();
    raise();
    activateWindow();

    _fadeInAnim.start();
    _slideInAnim.start();
    _progressBarAnim.start();

    _closeTimer.start(durationMs_);
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