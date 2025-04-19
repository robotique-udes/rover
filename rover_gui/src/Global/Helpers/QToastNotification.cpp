# include "QToastNotification.hpp"

QToastNotification::QToastNotification(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_):
    QWidget(parent_),
    _node(guiNode_),
    _fadeInAnim(this,"windowOpcaity"),
    _fadeOutAnim(this, "windowOpacity"),
    _slideAnim(this)
{
    _ui.setupUi(this);
    setupAnimations();
}

void QToastNotification::setupAnimations()
{
    _fadeInAnim.setDuration(300);
    _fadeInAnim.setStartValue(0);
    _fadeInAnim.setEndValue(1);

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

    _ui.textErrorMessage->setText(message);
    adjustSize();

    // Get screen size
    QPoint parentBottomRight = parentWidget()->geometry().bottomRight();
    int x = parentBottomRight.x() - width() - 20;
    int y = parentBottomRight.y() - height() - 20;
    move(x + parentWidget()->x(), y + parentWidget()->y());

    show();

    // Animate in
    _slideAnim.stop();
    _slideAnim.setStartValue(pos()+ QPoint(width(), 0));
    _slideAnim.setEndValue(pos());
    _slideAnim.setDuration(300);
    _slideAnim.start();

    // Auto-hide
    _closeTimer.start(durationMs);

}