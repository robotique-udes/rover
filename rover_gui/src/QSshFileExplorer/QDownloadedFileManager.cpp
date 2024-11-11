#include "QDownloadedFileManager.hpp"

std::list<QDownloadedFileManager::sFileInfo> QDownloadedFileManager::_availableFileList = {};

bool QDownloadedFileManager::sFileInfo::operator==(const sFileInfo& other) const
{
    return name == other.name && size == other.size;
}

QDownloadedFileManager::QDownloadedFileManager() {}

QDownloadedFileManager& QDownloadedFileManager::getInstance()
{
    static QDownloadedFileManager instance;
    return instance;
}

void QDownloadedFileManager::addFileToList(const std::string& fileName_, const uint64_t fileSize_)
{
    sFileInfo file = {.name = fileName_, .size = fileSize_};

    if (alreadyDownloaded(file) != eDownloadState::ALREADY_DOWNLOADED_OK)
    {
        _availableFileList.push_back(file);
    }
}

QDownloadedFileManager::eDownloadState QDownloadedFileManager::alreadyDownloaded(sFileInfo file_)
{
    return this->alreadyDownloaded(file_.name, file_.size);
}

QDownloadedFileManager::eDownloadState QDownloadedFileManager::alreadyDownloaded(const std::string& fileName_,
                                                                                 const uint64_t fileSize_)
{
    eDownloadState state = eDownloadState::NOT_DOWNLOADED;
    sFileInfo fileToFind = {.name = fileName_, .size = fileSize_};

    auto match = std::find_if(_availableFileList.begin(),
                              _availableFileList.end(),
                              [&fileToFind](const sFileInfo& x) { return x.name == fileToFind.name; });

    if (match != _availableFileList.end() && *match == fileToFind)
    {
        state = eDownloadState::ALREADY_DOWNLOADED_OK;
    }
    else if (match != _availableFileList.end())
    {
        state = eDownloadState::ALREADY_DOWNLOADED_ERROR;
    }

    return state;
}
