#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdlib>

#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QLabel>

#include "Global/Constant/StyleSheet.hpp"
#include "QSshFileExplorer/QFileTransferWidget.hpp"
#include "QSideBar/QSideBar.hpp"

class MainWindow : public QMainWindow
{
  public:
    explicit MainWindow(QWidget* parent_ = nullptr): QMainWindow(parent_), _fileTransferWidget(parent_), _sideBarWidget(parent_)
    {
        QWidget* centralWidget = new QWidget(this);
        QWidget* mainWidget = new QWidget(this);
        QHBoxLayout* layout = new QHBoxLayout(centralWidget);
        
        layout->addWidget(&_sideBarWidget);
        layout->addWidget(mainWidget);

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
