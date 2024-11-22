#include <QMessageBox>

#include "rovus_lib/macros.h"

namespace QHelper
{
    namespace QPopUp
    {
        QMessageBox::StandardButton sendQuestionPopUp(IN const std::string& title_,
                                                      IN const std::string& message_,
                                                      QFlags<QMessageBox::StandardButton> buttons_
                                                      = QMessageBox::StandardButton::NoButton);

        bool sendStringInputPopUp(IN const std::string& title_,
                                  IN const std::string& message_,
                                  OUT std::string& input_,
                                  bool passwordMode_);
    }  // namespace QPopUp

    namespace QTerminalCommand
    {
        bool blockingTerminalCommand(const std::string& command_,
                                     IN const QStringList& arguments_,
                                     OUT std::string& result_,
                                     const std::chrono::milliseconds timeout_ = std::chrono::milliseconds(1000));
    }  // namespace QTerminalCommand

    std::string getFileNameFromPath(const std::string& path_);
    std::string getFileExtension(const std::string& filename_);

}  // namespace QHelper
