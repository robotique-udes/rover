// ros
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdlib>

#include <QApplication>
#include <QMainWindow>

#include "Global/Constant/StyleSheet.hpp"
#include "QSshFileExplorer/QFileTransferWidget.hpp"

class MainWindow : public QMainWindow
{
  public:
    explicit MainWindow(QWidget* parent_ = nullptr): QMainWindow(parent_), _fileTransferWidget(parent_)
    {
        this->setCentralWidget(&_fileTransferWidget);
    }

    ~MainWindow() {}

  private:
    QFileTransferWidget _fileTransferWidget;
};

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    QApplication::setStyle("Fusion");
    app.setStyleSheet(STYLE_DARK_MODE);

    MainWindow page;
    page.setGeometry(0, 0, 1200, 600);

    page.show();

    return app.exec();
}
