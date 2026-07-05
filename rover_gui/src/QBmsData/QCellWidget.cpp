#include "QCellWidget.hpp"
#include "QBmsData.hpp"

constexpr const char* DEFAULT = "QWidget {"
                                    "background-color: #3c3f41;"
                                    "border-radius: 5px;"
                                    "padding: 5px 10px;"
                                    "}";

QCellWidget::QCellWidget(uint16_t cellIndex_, QBmsData* bmsGUI)
{
    QWidget* bmsDataContainer = new QWidget(bmsGUI->_ui.bmsData);
    bmsDataContainer->setStyleSheet(DEFAULT);
    bmsDataContainer->setFixedSize(CELL_WIDTH, CELL_HEIGHT);

    QVBoxLayout* containerLayout = new QVBoxLayout(bmsDataContainer);
    containerLayout->setContentsMargins(1, 1, 1, 1);
    containerLayout->setSpacing(1);

    QProgressBar* progressBar = new QProgressBar();
    progressBar->setRange(QBmsData::CELL_MIN_VOLT, QBmsData::CELL_MAX_VOLT);
    progressBar->setValue(QBmsData::CELL_MIN_VOLT);
    progressBar->setFormat("%v(%p%)");
    progressBar->setTextVisible(true);
    progressBar->setAlignment(Qt::AlignCenter);
    progressBar->setOrientation(Qt::Vertical);
    progressBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QLabel* titleLabel = new QLabel();
    titleLabel->setText(QString::fromStdString("Cell " + std::to_string(cellIndex_)));
    titleLabel->setAlignment(Qt::AlignCenter);

    QFont titleFont;
    titleFont.setFamily("Rajdhani");
    titleFont.setPointSize(20);
    titleFont.setBold(true);
    titleLabel->setFont(titleFont);

    containerLayout->addWidget(progressBar);
    containerLayout->addWidget(titleLabel);
    bmsGUI->_bmsDataTypes[cellIndex_] = {bmsDataContainer, progressBar};
}