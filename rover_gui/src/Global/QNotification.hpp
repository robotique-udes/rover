#include <QDialog>
#include <QVBoxLayout>

#include "UI_Notification.h"

class NotificationPopup : public QWidget
{
    Q_OBJECT

  public:
    explicit NotificationPopup(const QString& message, QWidget* parent = nullptr): QWidget(parent)
    {
        setWindowFlags(Qt::FramelessWindowHint);
        setAttribute(Qt::WA_ShowWithoutActivating);  // Don't grab focus

        _notificationWidget.setupUi(this);
    }

    void showAtCorner(QWidget* parent)
    {
        if (!parent)
            return;

        const int margin = 20;
        QSize parentSize = parent->size();
        QPoint globalPos = parent->mapToGlobal(QPoint(0, 0));

        // Position at the bottom-right corner of the parent window
        int x = globalPos.x() + parentSize.width() - width() - margin;
        int y = globalPos.y() + parentSize.height() - height() - margin;

        move(x, y);
        show();
    }

  private:
    Ui::Notification _notificationWidget;
};
