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
#include <QThread>

#include "rclcpp/rclcpp.hpp"

namespace QHelper
{

    QMessageBox::StandardButton QPopUp::sendQuestionPopUp(const std::string& title_,
                                                          const std::string& message_,
                                                          QMessageBox::StandardButtons buttons_)
    {
        QMessageBox::StandardButton userSelection = QMessageBox::NoButton;

        auto showDialog = [&]()
        {
            userSelection
                = QMessageBox::question(nullptr, QString::fromStdString(title_), QString::fromStdString(message_), buttons_);
        };

        if (QThread::currentThread() == QCoreApplication::instance()->thread())
        {
            // Already on main thread, call directly
            showDialog();
        }
        else
        {
            // Cross-thread: block calling thread until main thread completes
            QMetaObject::invokeMethod(QCoreApplication::instance(), showDialog, Qt::BlockingQueuedConnection);
        }

        return userSelection;
    }

    bool QPopUp::sendStringInputPopUp(const std::string& title_,
                                      const std::string& message_,
                                      OUT std::string& input_,
                                      const bool passwordMode_)
    {
        bool success = false;

        auto showDialog = [&]()
        {
            QInputDialog inputDialog;
            inputDialog.setWindowTitle(QString::fromStdString(title_));
            inputDialog.setLabelText(QString::fromStdString(message_));

            if (passwordMode_)
            {
                QLineEdit* pLineEdit = inputDialog.findChild<QLineEdit*>();
                if (pLineEdit)
                    pLineEdit->setEchoMode(QLineEdit::Password);
            }

            if (inputDialog.exec() == QDialog::Accepted)
            {
                input_ = inputDialog.textValue().toStdString();
                success = true;
            }
            else
            {
                input_ = "";
            }
        };

        if (QThread::currentThread() == QCoreApplication::instance()->thread())
        {
            showDialog();
        }
        else
        {
            QMetaObject::invokeMethod(QCoreApplication::instance(), showDialog, Qt::BlockingQueuedConnection);
        }

        return success;
    }

    bool QTerminalCommand::blockingTerminalCommand(const std::string& command_,
                                                   const QStringList& arguments_,
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
        QObject::connect(&timeoutTimer, &QTimer::timeout, &eventLoop, &QEventLoop::quit);
        timeoutTimer.start(timeout_);

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
        std::string fileExtension = "";
        size_t lastDotPos = filename_.find_last_of('.');

        // Avoid first dot for hidden folder
        if (lastDotPos != std::string::npos && lastDotPos != 0)
        {
            fileExtension = filename_.substr(lastDotPos + 1);
        }

        return fileExtension;
    }

    std::string getCurrentUserName(void)
    {
        QString username = QDir(QStandardPaths::writableLocation(QStandardPaths::HomeLocation)).dirName();
        return username.toStdString();
    }
}  // namespace QHelper
