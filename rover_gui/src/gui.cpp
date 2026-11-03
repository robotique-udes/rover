#include "Global/Constant/StyleSheet.hpp"

#include "MainWindow.hpp"
#include "SecondaryWindow.hpp"
#include "QScreenInhibitor.hpp"

#include <QApplication>
#include <QLabel>
#include <QWidget>

constexpr char WM_CLASS[] = "Rover Base";

int guiMain(int argc_, char* argv_[], const rclcpp::Node::SharedPtr& guiNode_);
void displayWindows(MainWindow& mainWindow_, SecondaryWindow& secondWindow_);
void nodeThreadFunc(const rclcpp::Node::SharedPtr& node);

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr guiNode = std::make_shared<rclcpp::Node>("gui_node");
    std::jthread rosThread(nodeThreadFunc, guiNode);

    int ret = guiMain(argc, argv, guiNode);

    rclcpp::shutdown();
    if (rosThread.joinable())
    {
        rosThread.join();
    }
    return ret;
}

/**
 * @brief Using a "2nd" main function to encapsulate all Qt elements in its own scope to make sure all destructors are called
 * after exiting the UI app independently of ros execution.
 *
 * @param argc_
 * @param argv_
 * @param guiNode_
 * @return int
 */
int guiMain(int argc_, char* argv_[], const rclcpp::Node::SharedPtr& guiNode_)
{
    QApplication app(argc_, argv_);
    QApplication::setApplicationName(WM_CLASS);
    QApplication::setStyle("Fusion");
    app.setStyleSheet(QString(Constants::Style::STYLE_DARK_MODE) + QString(Constants::Style::STATUS_STYLE));

    rclcpp::on_shutdown(
        []
        {
            QMetaObject::invokeMethod(qApp, &QCoreApplication::quit, Qt::QueuedConnection);
        });

    ScreenInhibitor screenInhibitor;

    MainWindow mainWindow(guiNode_);
    SecondaryWindow secondaryWindow(guiNode_);
    displayWindows(mainWindow, secondaryWindow);

    int ret = QApplication::exec();

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
            break;
        }

        default:
            mainWindow_.setGeometry(screens[0]->geometry());
            secondWindow_.setGeometry(screens[1]->geometry());
            mainWindow_.showMaximized();
            secondWindow_.showMaximized();
            break;
    }
}

void nodeThreadFunc(const rclcpp::Node::SharedPtr& node_)
{
    rclcpp::executors::MultiThreadedExecutor rosExecutor;
    rosExecutor.add_node(node_);
    rosExecutor.spin();

    rosExecutor.remove_node(node_);
}

#ifndef __INTELLISENSE__
// #include "gui.moc"
#endif  // __INTELLISENSE__
