#ifndef FOLDERS_HPP
#define FOLDERS_HPP
#include <string>
#include <sys/stat.h>
#include <vector>

#if defined(__linux__)

namespace Folders
{
    /**
     * @brief Checks if the folder exists
     *
     * @param path_ Path the the saving folder
     * @return true if it exists.
     * @return false if it doesn't or it isn't a folder
     */
    bool folderExists(const std::string& path_);

    /**
     * @brief Creates the desired folder with the necessary permissions for Linux
     *
     * @param path_ Path to the folder that needs to be created
     * @return true
     * @return false
     */
    bool createFolder(const std::string& path_);

    std::vector<std::string> splitpath(const std::string& path);
}  // namespace Folders

#endif  //(__linux__)

#endif  // FOLDERS_HPP