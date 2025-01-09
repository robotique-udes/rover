#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdlib>

#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QGridLayout>
#include <QWidget>
#include <QStackedWidget>
#include <QLabel>
#include <QComboBox>

#include "Global/Constant/StyleSheet.hpp"
#include "QSshFileExplorer/QFileTransferWidget.hpp"
#include "QSideBar/QSideBar.hpp"

class MainWindow : public QMainWindow
{
  Q_OBJECT

  public:
    explicit MainWindow(QWidget* parent_ = nullptr)
        : QMainWindow(parent_), _fileTransferWidget(parent_), _sideBarWidget(parent_)
    {
        QWidget* centralWidget = new QWidget(this);
        QHBoxLayout* layout = new QHBoxLayout(centralWidget);

        QWidget* dashboardWidget = new QWidget;
        QGridLayout* dashboardLayout = new QGridLayout(dashboardWidget);
        QLabel* dashboardLabel = new QLabel("Dashboard", dashboardWidget);
        dashboardLabel->setAlignment(Qt::AlignCenter);
        dashboardLayout->addWidget(dashboardLabel);

        QWidget* navigationWidget = new QWidget;
        QGridLayout* navigationLayout = new QGridLayout(navigationWidget);
        QLabel* navigationLabel = new QLabel("Navigation", navigationWidget);
        navigationLabel->setAlignment(Qt::AlignCenter);
        navigationLayout->addWidget(navigationLabel);

        QWidget* scienceWidget = new QWidget;
        QGridLayout* scienceLayout = new QGridLayout(scienceWidget);
        QLabel* scienceLabel = new QLabel("Science", scienceWidget);
        scienceLabel->setAlignment(Qt::AlignCenter);
        scienceLayout->addWidget(scienceLabel);

        QStackedWidget* stackedWidget = new QStackedWidget;
        stackedWidget->addWidget(dashboardWidget);
        stackedWidget->addWidget(navigationWidget);
        stackedWidget->addWidget(scienceWidget);

        connect(&_sideBarWidget, &QSideBar::switchPage, stackedWidget, &QStackedWidget::setCurrentIndex);

        layout->addWidget(&_sideBarWidget);
        layout->addWidget(stackedWidget);

        this->setCentralWidget(centralWidget);
    }

    ~MainWindow() {}

  private:
    QFileTransferWidget _fileTransferWidget;
    QSideBar _sideBarWidget;
};

class SecondaryWindow : public QMainWindow
{
  public:
    explicit SecondaryWindow(QWidget* parent_ = nullptr): QMainWindow(parent_)
    {
        auto centralWidget = new QWidget(this);
        auto layout = new QVBoxLayout(centralWidget);

        auto label = new QLabel("Future camera window", this);
        label->setAlignment(Qt::AlignCenter);

        layout->addWidget(label);

        this->setCentralWidget(centralWidget);
    }

    ~SecondaryWindow() {}
};

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    QApplication::setStyle("Fusion");
    app.setStyleSheet(STYLE_DARK_MODE);

    // Main Window
    MainWindow mainWindow;

    // Secondary Window
    SecondaryWindow secondaryWindow;
    secondaryWindow.setGeometry(1220, 0, 800, 600);

    mainWindow.showMaximized();
    secondaryWindow.show();

    return app.exec();
}

#include "main_gui.moc"