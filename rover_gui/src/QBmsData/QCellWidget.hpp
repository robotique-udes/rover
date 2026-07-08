#ifndef QCELLWIDGET_HPP
#define QCELLWIDGET_HPP

// QT
#include <QLabel>
#include <QProgressBar>
#include <QVBoxLayout>

class QProgressBar;

class QCellWidget : public QWidget
{
    Q_OBJECT

  public:
    static constexpr uint16_t CELL_WIDTH = 200U;
    static constexpr uint16_t CELL_HEIGHT = 400U;
    static constexpr uint16_t CELL_MIN_VOLT = 3000;
    static constexpr uint16_t CELL_MAX_VOLT = 4200;

    explicit QCellWidget(uint16_t cellIndex_, QWidget* parent_ = nullptr);
    void setVoltage(uint16_t volt_);
    void setCellContainerSize(uint16_t width_, uint16_t height_);

  private:
    QProgressBar* _progressBar;
};

#endif