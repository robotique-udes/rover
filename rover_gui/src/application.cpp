// ros
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// Qt
#include <QApplication>
#include <QMainWindow>
#include <QStatusBar>
#include <QFileSystemModel>
#include <QString>

// Ui files
#include "ui_file_transfer_widget.h"

#define DEFAULT_ROOT_PATH ament_index_cpp::get_package_share_directory("rover_gui") + "/../../../../src/rover"
#define DEFAULT_PATH DEFAULT_ROOT_PATH + "/rover_gui/"
// #define DEFAULT_PATH std::string("/home/phil")

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget *parent = nullptr) : QMainWindow(parent)
    {

        QWidget* fileTransfer = new QWidget(this);
        ui.setupUi(fileTransfer);

        QFileSystemModel *model = new QFileSystemModel;

        model->setRootPath(QString::fromStdString(DEFAULT_ROOT_PATH));

        ui.tw_localFileExplorer->setModel(model);
        ui.tw_localFileExplorer->setRootIndex(model->index(QString::fromStdString(DEFAULT_PATH)));

        this->setCentralWidget(fileTransfer);
    }

    ~MainWindow() {}

private:
    Ui::FileTransfer ui;
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    MainWindow page;

    page.show();

    return app.exec();
}
