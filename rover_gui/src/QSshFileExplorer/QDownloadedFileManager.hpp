#ifndef __QDOWNLOADED_FILE_MANAGER_HPP__
#define __QDOWNLOADED_FILE_MANAGER_HPP__

#include "../Global/QTmpFolderManager.hpp"

#include "rclcpp/rclcpp.hpp"
#include <unordered_map>

class QDownloadedFileManager
{
  public:
    enum class eDownloadState : size_t
    {
        NOT_DOWNLOADED,
        ALREADY_DOWNLOADED_OK,
        ALREADY_DOWNLOADED_SIZE_MISSMATCH
    };

    struct sFileInfo
    {
        std::string name = "";
        uint64_t size = 0u;

        bool operator==(const sFileInfo& other) const;
    };

    QDownloadedFileManager();
    ~QDownloadedFileManager() = default;

    QDownloadedFileManager(const QDownloadedFileManager&) = delete;
    QDownloadedFileManager& operator=(const QDownloadedFileManager&) = delete;

    static QDownloadedFileManager& getInstance();
    void addFileToList(const std::string& fileName_, const uint64_t fileSize_);
    eDownloadState alreadyDownloaded(sFileInfo file_);
    eDownloadState alreadyDownloaded(const std::string& fileName_, const uint64_t fileSize_);
    #warning TODO
    // bool getFilePath(sFileInfo file_);

  private:
    static std::unordered_map<decltype(sFileInfo::name), decltype(sFileInfo::size)> _availableFileList;
};

#endif  // __QDOWNLOADED_FILE_MANAGER_HPP__
