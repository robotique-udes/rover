#ifndef QCELLWIDGET_HPP
#define QCELLWIDGET_HPP

// QT
#include <QProgressBar>

class QCellWidget : public QWidget
{
    Q_OBJECT

  public:
    static constexpr uint16_t CELL_MIN_VOLT_MV = 3000;
    static constexpr uint16_t CELL_MAX_VOLT_MV = 4200;

    explicit QCellWidget(uint16_t cellIndex_);
    void setVoltage(uint16_t volt_);
    void setCellContainerSize(uint16_t width_, uint16_t height_);

  private:
    QProgressBar* _progressBar;
};

#endif