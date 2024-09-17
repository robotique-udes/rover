// ros
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// Qt
#include <QMainWindow>

// Ui files
#include "ui_file_transfer_widget.h"

// Custom objects
#include "q_ssh_file_explorer.hpp"

// Others
#include <cstdlib>

#define DEFAULT_ROOT_PATH ament_index_cpp::get_package_share_directory("rover_gui") + "/../../../../src/rover"
#define DEFAULT_PATH DEFAULT_ROOT_PATH + "/rover_gui/"
// #define DEFAULT_PATH std::string("/home/phil")

class FileTransferWidget : public QWidget
{
public:
    FileTransferWidget()
    {
        ui.setupUi(this);
        roverFileSystem = new QSshFileExplorer(this, ui.tv_rover);
    }

    ~FileTransferWidget()
    {
        if (roverFileSystem)
        {
            delete roverFileSystem;
        }
    }

private:
    Ui::FileTransfer ui;
    QSshFileExplorer *roverFileSystem = NULL;
};

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget *parent = nullptr) : QMainWindow(parent)
    {
        FileTransferWidget *fileTransferWidget = new FileTransferWidget();
        this->setCentralWidget(fileTransferWidget);
    }

    ~MainWindow() {}
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindow page;
    page.setGeometry(0, 0, 1200, 600);

    page.show();

    return app.exec();
}
