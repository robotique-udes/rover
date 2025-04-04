#ifndef QPLAYPAUSEBUTTON_HPP
#define QPLAYPAUSEBUTTON_HPP

#include <QPushButton>
#include <QPropertyAnimation>
#include <QPainter>
#include <QColor>

// Custom button class that toggles between play and pause states with animation
class QPlayPauseButton : public QPushButton
{
    Q_OBJECT
    Q_PROPERTY(qreal animationProgress READ animationProgress WRITE setAnimationProgress)

public:
    explicit QPlayPauseButton(QWidget* parent = nullptr) : 
        QPushButton(parent),
        _isPlaying(false),
        _animationProgress(0.0)
    {
        // Set fixed size for better appearance
        setFixedSize(40, 40);
        
        // Create animation
        _animation = new QPropertyAnimation(this, "animationProgress");
        _animation->setDuration(300);
        _animation->setEasingCurve(QEasingCurve::OutCubic);
        
        // Connect toggle signal
        connect(this, &QPushButton::clicked, this, &QPlayPauseButton::toggle);
        
        // Set tooltip based on initial state
        updateTooltip();
    }
    
    ~QPlayPauseButton() {
        delete _animation;
    }
    
    bool isPlaying() const {
        return _isPlaying;
    }
    
    qreal animationProgress() const {
        return _animationProgress;
    }
    
    void setAnimationProgress(qreal progress) {
        _animationProgress = progress;
        update(); // Trigger repaint
    }

public slots:
    void toggle() {
        _isPlaying = !_isPlaying;
        
        // Set animation target value based on new state
        _animation->setStartValue(_isPlaying ? 0.0 : 1.0);
        _animation->setEndValue(_isPlaying ? 1.0 : 0.0);
        
        // Start animation
        _animation->start();
        
        // Update tooltip
        updateTooltip();
        
        // Emit signals based on new state
        if (_isPlaying) {
            emit playClicked();
        } else {
            emit pauseClicked();
        }
    }
    
    void setPlaying(bool playing) {
        if (_isPlaying != playing) {
            toggle();
        }
    }

signals:
    void playClicked();
    void pauseClicked();

protected:
    void paintEvent(QPaintEvent* event) override {
        Q_UNUSED(event);
        
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);
        
        // Draw background
        QColor bgColor = isEnabled() ? QColor(60, 60, 60) : QColor(40, 40, 40);
        painter.setBrush(bgColor);
        painter.setPen(Qt::NoPen);
        painter.drawRoundedRect(rect(), 5, 5);
        
        // Set foreground color - always using white with different brightness levels based on state
        QColor fgColor;
        if (!isEnabled()) {
            fgColor = QColor(150, 150, 150); // Dimmed white when disabled
        } else if (isDown()) {
            fgColor = QColor(200, 200, 200); // Slightly dimmer white when pressed
        } else if (underMouse()) {
            fgColor = QColor(255, 255, 255); // Pure white when hovered
        } else {
            fgColor = QColor(230, 230, 230); // Default slightly off-white
        }
        
        painter.setBrush(fgColor);
        painter.setPen(Qt::NoPen);
        
        // Calculate dimensions
        const int margin = 10;
        QRect contentRect = rect().adjusted(margin, margin, -margin, -margin);
        
        // Mix between play and pause icons based on animation progress
        if (_animationProgress < 0.5) {
            // More like a play triangle
            // Using animation progress directly instead of calculating unused playRatio
            
            // Draw a triangle morphing to bars
            QPolygonF polygon;
            
            // Left point (moves right)
            polygon << QPointF(contentRect.left() + contentRect.width() * _animationProgress,
                              contentRect.top());
            
            // Right point (fixed)
            polygon << QPointF(contentRect.right(),
                              contentRect.top() + contentRect.height() / 2);
            
            // Bottom point (moves up)
            polygon << QPointF(contentRect.left() + contentRect.width() * _animationProgress,
                              contentRect.bottom());
            
            painter.drawPolygon(polygon);
            
            // Draw emerging right bar (gets more visible)
            QRectF rightBar(contentRect.right() - contentRect.width() * 0.2,
                           contentRect.top(),
                           contentRect.width() * 0.2,
                           contentRect.height());
            
            QColor rightBarColor = fgColor;
            rightBarColor.setAlphaF(_animationProgress * 2.0);
            painter.setBrush(rightBarColor);
            painter.drawRoundedRect(rightBar, 2, 2);
        } else {
            // More like pause bars
            qreal pauseRatio = (_animationProgress - 0.5) * 2.0;
            
            // Draw left bar
            QRectF leftBar(contentRect.left(),
                          contentRect.top(),
                          contentRect.width() * 0.2,
                          contentRect.height());
            painter.drawRoundedRect(leftBar, 2, 2);
            
            // Draw right bar
            QRectF rightBar(contentRect.right() - contentRect.width() * 0.2,
                           contentRect.top(),
                           contentRect.width() * 0.2,
                           contentRect.height());
            painter.drawRoundedRect(rightBar, 2, 2);
            
            // Draw fading triangle
            QColor triangleColor = fgColor;
            triangleColor.setAlphaF(1.0 - pauseRatio);
            painter.setBrush(triangleColor);
            
            QPolygonF polygon;
            polygon << QPointF(contentRect.left(), contentRect.top());
            polygon << QPointF(contentRect.right(), contentRect.top() + contentRect.height() / 2);
            polygon << QPointF(contentRect.left(), contentRect.bottom());
            
            painter.drawPolygon(polygon);
        }
    }

private:
    bool _isPlaying;
    qreal _animationProgress;
    QPropertyAnimation* _animation;
    
    void updateTooltip() {
        setToolTip(_isPlaying ? "Stop" : "Play");
    }
};

#endif // QPLAYPAUSEBUTTON_HPP