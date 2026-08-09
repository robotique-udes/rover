#include "QCellWidget.hpp"
#include "Global/Constant/StyleSheet.hpp"
// QT
#include <QVBoxLayout>
#include <QLabel>

QCellWidget::QCellWidget(uint16_t cellIndex_)
{
    setStyleSheet(Constants::Style::BMS_STYLE);

    QVBoxLayout* layout = new QVBoxLayout(this);
    layout->setContentsMargins(1, 1, 1, 1);
    layout->setSpacing(1);

    _progressBar = new QProgressBar();
    _progressBar->setRange(CELL_MIN_VOLT_MV, CELL_MAX_VOLT_MV);
    _progressBar->setValue(CELL_MIN_VOLT_MV);
    _progressBar->setFormat("%v mV(%p%)");
    _progressBar->setTextVisible(true);
    _progressBar->setAlignment(Qt::AlignCenter);
    _progressBar->setOrientation(Qt::Vertical);
    _progressBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QLabel* titleLabel = new QLabel();
    titleLabel->setText(QString::fromStdString("Cell " + std::to_string(cellIndex_)));
    titleLabel->setAlignment(Qt::AlignCenter);

    QFont titleFont;
    titleFont.setFamily("Rajdhani");
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);

    layout->addWidget(_progressBar);
    layout->addWidget(titleLabel);
}

void QCellWidget::setVoltage(uint16_t volt_)
{
    _progressBar->setValue(volt_);
}

void QCellWidget::setCellContainerSize(uint16_t width_, uint16_t height_)
{
    setFixedSize(width_, height_);
}