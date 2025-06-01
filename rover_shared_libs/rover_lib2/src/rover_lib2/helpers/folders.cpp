#include "folders.hpp"

#if defined(__linux__)
#include "log.hpp"
#include <sstream>
#include <sys/stat.h>

constexpr size_t MAX_SUBDIR_COUNT = 1024UL;

DEFINE_LOG_NODE(FoldersHelper, Logger::eNodeState::OFF);

bool Folders::folderExists(const std::string& path_)
{
    struct stat fileInfo;

    if (stat(path_.c_str(), &fileInfo) != 0)
    {
        return false;
    }
    else
    {
        return (fileInfo.st_mode & S_IFDIR);
    }
}

bool Folders::createFolder(const std::string& path_)
{
    std::vector<std::string> subdirectories = Folders::splitPath(path_);
    std::string currentDirectory;
    for (const std::string& subdirectory : subdirectories)
    {
        currentDirectory += '/' + subdirectory;
        if (!Folders::folderExists(currentDirectory))
        {
            if (mkdir(currentDirectory.c_str(), 0775) != 0)
            {
                std::string errorMessage = "Failed to create folder for path: " + currentDirectory;
                LOG_INFO(Logger::Nodes::FoldersHelper, errorMessage.c_str());
                return false;
            }
        }
    }

    return true;
}

std::vector<std::string> Folders::splitPath(const std::string& path_)
{
    char delimiter = '/';
    std::vector<std::string> subdirectories;
    std::stringstream stringstream(path_);
    std::string sub;
    size_t count = 0;

    for (std::string sub; (count < MAX_SUBDIR_COUNT && std::getline(stringstream, sub, delimiter));)
    {
        if (!sub.empty())
        {
            subdirectories.push_back(sub);
            count++;
        }
    }

    return subdirectories;
}

#endif  //(__linux__)
