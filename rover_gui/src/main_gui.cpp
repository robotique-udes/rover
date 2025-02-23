// ros
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <cstdlib>

#include <QApplication>
#include <QMainWindow>
#include <QLoggingCategory>
#include "QRtspPlayer/QRtspPlayerWidget.hpp"

#include "Global/Constant/StyleSheet.hpp"
#include "QSshFileExplorer/QFileTransferWidget.hpp"

class MainWindow : public QMainWindow
{
  public:
    explicit MainWindow(QWidget* parent_ = nullptr): QMainWindow(parent_), _rtspPlayerWidget(new RtspPlayerWidget(this))
    {
        this->setCentralWidget(_rtspPlayerWidget);
    }

    ~MainWindow() {}

  private:
    RtspPlayerWidget* _rtspPlayerWidget;
};

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    QLoggingCategory::setFilterRules("*.debug=false");  // This will show info, warning, and error messages

    QApplication::setStyle("Fusion");
    app.setStyleSheet(STYLE_DARK_MODE);

    MainWindow page;
    page.setGeometry(0, 0, 1200, 600);

    page.show();

    return app.exec();
}
