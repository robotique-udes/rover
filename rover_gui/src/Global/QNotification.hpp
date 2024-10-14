#include <QDialog>
#include <QLabel>
#include <QTimer>
#include <QVBoxLayout>

class NotificationPopup : public QDialog
{
    Q_OBJECT

  public:
    explicit NotificationPopup(const QString& message, QWidget* parent = nullptr): QDialog(parent)
    {
        setWindowFlags(Qt::FramelessWindowHint);

        setAttribute(Qt::WA_TranslucentBackground);  // Transparent background
        setAttribute(Qt::WA_ShowWithoutActivating);  // Don't grab focus

        QVBoxLayout* layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);

        QLabel* label = new QLabel(message, this);
        label->setStyleSheet("color: white; background-color: rgba(0, 0, 0, 160); "
                             "padding: 10px; border-radius: 5px;");

        layout->addWidget(label);

        setLayout(layout);
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
};
