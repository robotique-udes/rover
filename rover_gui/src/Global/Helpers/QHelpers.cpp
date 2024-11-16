#include "QHelpers.hpp"

#include <QApplication>
#include <QEventLoop>
#include <QFileInfo>
#include <QInputDialog>
#include <QMessageBox>
#include <QMetaObject>
#include <QProcess>
#include <QString>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"

#include "rovus_lib/macros.h"

namespace QHelper
{

    QMessageBox::StandardButton QPopUp::sendQuestionPopUp(IN const std::string& title_,
                                                          IN const std::string& message_,
                                                          QFlags<QMessageBox::StandardButton> buttons_)
    {
        QEventLoop waitForAnswerLoop;
        QMessageBox::StandardButton userSelection = QMessageBox::StandardButton::NoButton;

        QCoreApplication* pApp = QApplication::instance();
        if (pApp)
        {
            QMetaObject::invokeMethod(pApp,
                                      [&]()
                                      {
                                          userSelection = static_cast<QMessageBox::StandardButton>(
                                              QMessageBox::question(nullptr,
                                                                    QString::fromStdString(title_),
                                                                    QString::fromStdString(message_),
                                                                    buttons_));
                                          waitForAnswerLoop.quit();
                                      });
            waitForAnswerLoop.exec();
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "QApplication returned null, something is very wrong");
        }

        return userSelection;
    }

    bool QPopUp::sendStringInputPopUp(IN const std::string& title_,
                                      IN const std::string& message_,
                                      OUT std::string& input_,
                                      bool passwordMode_)
    {
        bool success = true;
        QEventLoop waitForAnswerLoop;

        QCoreApplication* pApp = QApplication::instance();
        if (success && !pApp)
        {
            RCLCPP_ERROR(rclcpp::get_logger("GUI"), "QApplication returned null, something is very wrong");
            success = false;
        }
        else
        {
            QMetaObject::invokeMethod(pApp,
                                      [&]()
                                      {
                                          QInputDialog inputDialog;
                                          inputDialog.setWindowTitle(QString::fromStdString(title_));
                                          inputDialog.setLabelText(QString::fromStdString(message_));

                                          if (passwordMode_)
                                          {
                                              QLineEdit* plineEdit = inputDialog.findChild<QLineEdit*>();
                                              if (plineEdit)
                                              {
                                                  plineEdit->setEchoMode(QLineEdit::Password);
                                              }
                                          }

                                          if (inputDialog.exec() == QDialog::Accepted)
                                          {
                                              input_ = inputDialog.textValue().toStdString();
                                          }
                                          else
                                          {
                                              input_ = "";
                                              success = false;
                                          }

                                          waitForAnswerLoop.quit();
                                      });

            waitForAnswerLoop.exec();
        }

        return success;
    }

    bool QTerminalCommand::blockingTerminalCommand(IN const std::string& command_,
                                                   IN const QStringList& arguments_,
                                                   OUT std::string& result_,
                                                   const std::chrono::milliseconds timeout_)
    {
        bool success = true;
        QProcess process;
        QEventLoop eventLoop;

        QApplication::setOverrideCursor(Qt::WaitCursor);
        process.start(QString::fromStdString(command_), arguments_);
        QObject::connect(&process, &QProcess::finished, &eventLoop, &QEventLoop::quit);

        QTimer timeoutTimer;
        timeoutTimer.setSingleShot(true);
        timeoutTimer.start(timeout_);
        QObject::connect(&timeoutTimer, &QTimer::timeout, &eventLoop, &QEventLoop::quit);

        eventLoop.exec();

        QApplication::restoreOverrideCursor();
        if (timeoutTimer.isActive())
        {
            result_ = QString(process.readAll()).toStdString();
        }
        else
        {
            result_ = "";
            success = false;
        }

        return success;
    }

    std::string getFileNameFromPath(const std::string& path_)
    {
        return QFileInfo(QString::fromStdString(path_)).fileName().toStdString();
    }

    std::string getFileExtension(const std::string& filename_)
    {
#warning TODO: Missing multiple checks
        size_t lastDotPos = filename_.find_last_of('.');

        // Avoid first dot for hidden folder
        if (lastDotPos != std::string::npos && lastDotPos != 0)
        {
            return filename_.substr(lastDotPos + 1);
        }

        return "";
    }
}  // namespace QHelper
