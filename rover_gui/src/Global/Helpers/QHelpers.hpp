#include <QMessageBox>
#include <QStandardPaths>
#include <QDir>

#include "rovus_lib/macros.h"

namespace QHelper
{
    /**
     * @brief Namespace with helpers to create blocking popups from any thread.
     * Won't block the ui main thread
     *
     */
    namespace QPopUp
    {
        /**
         * @brief Helper to send a sync (blocking) question popup to the user
         * from any thread. The user input is then returned when the user makes
         * his choice.
         *
         * @param title_ Popup title
         * @param message_ Popup message
         * @param buttons_ Buttons the user will be able to click
         * @return QMessageBox::StandardButton User selection from the passed
         * "buttons_"
         */
        QMessageBox::StandardButton sendQuestionPopUp(const std::string& title_,
                                                      const std::string& message_,
                                                      QMessageBox::StandardButtons buttons_
                                                      = QMessageBox::StandardButton::NoButton);

        /**
         * @brief Helper to send a sync (blocking) text input popup to the user
         * from any thread. The user input is then returned when the user press
         * the "ok" button
         *
         * @param title_ Popup title
         * @param message_ Popup message
         * @param input_ [OUT] Used to return the user input
         * @param passwordMode_ Hides the text input display to hide
         * passwords. *Warning:* passwords input are still returned in plain text)
         * @return true
         * @return false
         */
        bool sendStringInputPopUp(const std::string& title_,
                                  const std::string& message_,
                                  OUT std::string& input_,
                                  const bool passwordMode_ = false);
    }  // namespace QPopUp

    namespace QTerminalCommand
    {
        /**
         * @brief Sends a command into a bash terminal. Please use only in last
         * resort.
         *
         * @param command_
         * @param arguments_
         * @param result_ [OUT] Used to return the result of the command as a
         * string
         * @param timeout_ Command execution time before failing
         * @return success
         */
        bool blockingTerminalCommand(const std::string& command_,
                                     const QStringList& arguments_,
                                     OUT std::string& result_,
                                     const std::chrono::milliseconds timeout_ = std::chrono::milliseconds(1000));
    }  // namespace QTerminalCommand

    /**
     * @brief Get the filename from a file path
     *
     * @param path_
     * @return std::string
     */
    std::string getFileNameFromPath(const std::string& path_);

    /**
     * @brief Get the extension from a file name
     *
     * @param filename_
     * @return std::string
     */
    std::string getFileExtension(const std::string& filename_);

    /**
     * @brief Gets the username of the one executing the app
     *
     * @return std::string
     */
    std::string getCurrentUserName(void);

}  // namespace QHelper
