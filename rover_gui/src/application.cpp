// ros
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdlib>

#include <QMainWindow>
#include <QApplication>

// All .ui file must be included here to be generated as .h files
#include "ui_file_transfer_widget.h"

#include "style_sheet.hpp"
#include "q_file_transfer_widget.hpp"

// #define DEFAULT_ROOT_PATH ament_index_cpp::get_package_share_directory("rover_gui") + "/../../../../src/rover"
// #define DEFAULT_PATH DEFAULT_ROOT_PATH + "/rover_gui/"
// #define DEFAULT_PATH std::string("/home/phil")

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget *parent = nullptr) : QMainWindow(parent)
    {
        QFileTransferWidget *fileTransferWidget = new QFileTransferWidget();
        this->setCentralWidget(fileTransferWidget);
    }

    ~MainWindow() {}
};

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    // Windows decoration in darkmode when using wayland 
    QApplication::setStyle("Fusion");
    app.setStyleSheet(STYLE_DARK_MODE);

    MainWindow page;
    page.setGeometry(0, 0, 1200, 600);

    page.show();

    return app.exec();
}
