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
#include "QDashboard/QDashboard.hpp"
#include "QNavigation/QNavigation.hpp"

class MainWindow : public QMainWindow
{
  Q_OBJECT

  public:
    explicit MainWindow(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_ = nullptr)
        : QMainWindow(parent_),
          _fileTransferWidget(parent_),
          _sideBarWidget(parent_),
          _dashboardWidget(guiNode_, parent_),
          _navigationWidget(guiNode_, parent_)
    {
        QWidget* centralWidget = new QWidget(this);
        QHBoxLayout* layout = new QHBoxLayout(centralWidget);

        QStackedWidget* stackedWidget = new QStackedWidget;
        stackedWidget->addWidget(&_dashboardWidget);
        stackedWidget->addWidget(&_navigationWidget);
        stackedWidget->addWidget(&_fileTransferWidget);

        connect(&_sideBarWidget, &QSideBar::switchPage, stackedWidget, &QStackedWidget::setCurrentIndex);

        layout->addWidget(&_sideBarWidget);
        layout->addWidget(stackedWidget);

        this->setCentralWidget(centralWidget);
    }

    ~MainWindow() {}

  private:
    QFileTransferWidget _fileTransferWidget;
    QSideBar _sideBarWidget;
    QDashboard _dashboardWidget;
    QNavigation _navigationWidget;
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

void node_spin_thread(std::shared_ptr<rclcpp::Node> node)
{
    RCLCPP_INFO(node->get_logger(), "GUI Node started");

    rclcpp::spin(node);
    rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto guiNode = std::make_shared<rclcpp::Node>("gui_node");

    std::thread rosThread(node_spin_thread, guiNode);

    QApplication app(argc, argv);
    QApplication::setStyle("Fusion");
    app.setStyleSheet(STYLE_DARK_MODE);

    MainWindow mainWindow(guiNode);
    SecondaryWindow secondaryWindow;
    secondaryWindow.setGeometry(1220, 0, 800, 600);

    mainWindow.showMaximized();
    secondaryWindow.show();

    int ret = app.exec();

    if (rosThread.joinable()) {
        rosThread.join();
    }

    rclcpp::shutdown();
    return ret;
}

#include "main_gui.moc"