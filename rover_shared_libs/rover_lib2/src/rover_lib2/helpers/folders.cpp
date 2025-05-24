#include "folders.hpp"
#include <sstream>

#if defined(__linux__)

bool Folders::folderExists(const std::string& path_)
{
    struct stat fileInfo;

    if (stat(path_.c_str(), &fileInfo) != 0)
    {
        return false;
    }

    if (fileInfo.st_mode & S_IFDIR)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Folders::createFolder(const std::string& path_)
{
    std::vector<std::string> subdirectories = Folders::splitpath(path_);
    std::string currentDirectory;
    for (const std::string& subdirectory : subdirectories)
    {
        currentDirectory += '/' + subdirectory;
        if (!Folders::folderExists(currentDirectory))
        {
            if (mkdir(currentDirectory.c_str(), 0775) != 0)
            {
                return false;
            }
        }
    }

    return true;
}

std::vector<std::string> Folders::splitpath(const std::string& path_)
{
    char delimiter = '/';
    std::vector<std::string> subdirectories;
    std::stringstream stringstream(path_);
    std::string sub;
    while (std::getline(stringstream, sub, delimiter))
    {
        if (!sub.empty())
        {
            subdirectories.push_back(sub);
        }
    }
    return subdirectories;
}

#endif  //(__linux__)