#include "QToastNotification.hpp"
#include <QScreen>

QToastNotification::QToastNotification():QWidget(nullptr),
    _fadeInAnim(),
    _fadeOutAnim(),
    _slideAnim()
{
    qDebug() << "constructor";
    setWindowFlags(Qt::FramelessWindowHint | Qt::ToolTip);
    setAttribute(Qt::WA_TranslucentBackground);
    setAttribute(Qt::WA_ShowWithoutActivating);
    _ui.setupUi(this);
    _slideAnim.setTargetObject(this);
    _slideAnim.setPropertyName("pos");
    setupAnimations();
    qDebug() << "destructor";
    _ui.frame->setStyleSheet("background-color: #4d4d4d;;"  // Blue background
    "border-radius: 15px;"        // Rounded corners (adjust px for sharpness)
    "border: 2px solid #2980b9;"  // Optional border
);
    
}
QToastNotification& QToastNotification::getInstance()
{
    static QToastNotification instance;
    return instance;   
}

void QToastNotification::setupAnimations()
{
    _fadeInAnim.setTargetObject(this);
    _fadeInAnim.setPropertyName("windowOpacity");
    _fadeInAnim.setDuration(300);
    _fadeInAnim.setStartValue(0);
    _fadeInAnim.setEndValue(1);

    _fadeOutAnim.setTargetObject(this);
    _fadeOutAnim.setPropertyName("windowOpacity");
    _fadeOutAnim.setDuration(300);
    _fadeOutAnim.setStartValue(1);
    _fadeOutAnim.setEndValue(0);

    connect(&_fadeOutAnim, &QPropertyAnimation::finished, this, &QWidget::hide);

    _closeTimer.setSingleShot(true);
    connect(&_closeTimer, &QTimer::timeout, this, [this]() {
    _fadeOutAnim.start();  // fades out after timeout
    });    

}

void QToastNotification::showMessage(const QString& message, int durationMs)
{

    if (!_ui.textErrorMessage) {
    qDebug() << "textErrorMessage is null!";
    return;
    }
    _ui.textErrorMessage->setText(message);
    adjustSize();

    // Get screen size
    #warning clean /////
    QList<QScreen*> screens = QGuiApplication::screens();
    QScreen* targetScreen = nullptr;

    // Choose the secondary screen if available, else fallback to primary
    if (screens.size() >= 2) {
        targetScreen = screens[1]; // Index 1 = secondary screen
    } else {
        targetScreen = QGuiApplication::primaryScreen();
    }
    ////
    QRect screenRect = targetScreen->availableGeometry();
    int offscreenX = screenRect.right() + 20;
    int targetX = screenRect.right() - width() - 20;
    int targetY = screenRect.bottom() - height() - 40;

    move(offscreenX, targetY);  // start offscreen
    show();

    // Animate in
    setWindowOpacity(0);
    _fadeInAnim.start();

    _slideAnim.stop();
    _slideAnim.setStartValue(QPoint(offscreenX, targetY));
    _slideAnim.setEndValue(QPoint(targetX, targetY));
    _slideAnim.setDuration(300);
    _slideAnim.start();

    _closeTimer.start(durationMs);

}