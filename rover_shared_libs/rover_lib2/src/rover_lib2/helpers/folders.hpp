#ifndef ROVER_LIB2_HELPERS_FOLDERS_HPP
#define ROVER_LIB2_HELPERS_FOLDERS_HPP

#if defined(__linux__)
#include <string>
#include <vector>

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

    /**
     * @brief Splits a path into a list of directories as strings
     *
     * @param path
     * @return std::vector<std::string>
     */
    std::vector<std::string> splitPath(const std::string& path);
}  // namespace Folders

#endif  //(__linux__)

#endif  // ROVER_LIB2_HELPERS_FOLDERS_HPP
