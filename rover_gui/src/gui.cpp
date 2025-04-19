#include "Global/Constant/StyleSheet.hpp"
#include "MainWindow.hpp"
#include "SecondaryWindow.hpp"

#include <QApplication>
#include <QLabel>
#include <QWidget>

#include <QProcess>

constexpr char WM_CLASS[] = "Rover Base";

void displayWindows(MainWindow& mainWindow_, SecondaryWindow& secondWindow_);
void nodeThreadFunc(std::shared_ptr<rclcpp::Node> node);
void forwardPrints(QProcess& process_);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> guiNode = std::make_shared<rclcpp::Node>("gui_node");
    std::thread rosThread(nodeThreadFunc, guiNode);

    QApplication app(argc, argv);
    app.setApplicationName(WM_CLASS);
    QApplication::setStyle("Fusion");
    app.setStyleSheet(STYLE_DARK_MODE);

    MainWindow mainWindow(guiNode);
    SecondaryWindow secondaryWindow(guiNode);
    displayWindows(mainWindow, secondaryWindow);

    QProcess rosProcess;
    // clang-format off
    QObject::connect(&rosProcess, &QProcess::readyReadStandardOutput, [&](){ forwardPrints(rosProcess); });
    QObject::connect(&rosProcess, &QProcess::readyReadStandardError, [&](){ forwardPrints(rosProcess); });
    // clang-format on

    rosProcess.start("bash",
                     QStringList() << "-c"
                                   << "source ~/.bashrc && ros2 launch rover_msgs base.launch.py");

    int ret = app.exec();
    rosProcess.terminate();
    if (!rosProcess.waitForFinished(3000))
    {
        rosProcess.kill();
    }

    rclcpp::shutdown();

    if (rosThread.joinable())
    {
        rosThread.join();
    }

    rclcpp::shutdown();
    return ret;
}

void displayWindows(MainWindow& mainWindow_, SecondaryWindow& secondWindow_)
{
    QList<QScreen*> screens = QGuiApplication::screens();

    switch (screens.size())
    {
        case 0:
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Can't show GUI without screens");
            break;
        case 1:
        {
            QRect screenGeometry = screens[0]->geometry();
            int screenWidth = screenGeometry.width();
            int screenHeight = screenGeometry.height();

            mainWindow_.setGeometry(screenGeometry.x(), screenGeometry.y(), screenWidth / 2, screenHeight);
            mainWindow_.show();
            secondWindow_.setGeometry(screenGeometry.x() + screenWidth / 2, screenGeometry.y(), screenWidth / 2, screenHeight);
            secondWindow_.show();
            QTimer::singleShot(1000, [] {
            QToastNotification::getInstance().showMessage("Hello from the toast!", 3000);
            });
            #warning enleveee
            break;
        }

        default:
            mainWindow_.setGeometry(screens[0]->geometry());
            secondWindow_.setGeometry(screens[1]->geometry());
            mainWindow_.showMaximized();
            secondWindow_.showMaximized();
             QTimer::singleShot(1000, [] {
            QToastNotification::getInstance().showMessage("Hello from the toast!", 3000);
            });
            #warning enleveee
            break;
    }
}

void nodeThreadFunc(std::shared_ptr<rclcpp::Node> node_)
{
    rclcpp::executors::SingleThreadedExecutor rosExecutor;
    rosExecutor.add_node(node_);
    rosExecutor.spin();

    rosExecutor.remove_node(node_);
}

void forwardPrints(QProcess& process_)
{
    std::cout << process_.readAllStandardOutput().toStdString();
    std::cerr << process_.readAllStandardError().toStdString();
}

#ifndef __INTELLISENSE__
#include "gui.moc"
#endif  // __INTELLISENSE__
