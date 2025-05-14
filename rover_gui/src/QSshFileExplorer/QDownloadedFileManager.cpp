#include "QDownloadedFileManager.hpp"

#include "Global/QTmpFolderManager.hpp"

std::unordered_map<decltype(QDownloadedFileManager::sFileInfo::name), decltype(QDownloadedFileManager::sFileInfo::size)>
    QDownloadedFileManager::_availableFileList = {};

bool QDownloadedFileManager::sFileInfo::operator==(const sFileInfo& other) const
{
    return name == other.name && size == other.size;
}

QDownloadedFileManager::QDownloadedFileManager() {}

QDownloadedFileManager& QDownloadedFileManager::getInstance(void)
{
    static QDownloadedFileManager instance;
    return instance;
}

void QDownloadedFileManager::addFileToList(const std::string& fileName_, const uint64_t fileSize_)
{
    sFileInfo file = {.name = fileName_, .size = fileSize_};
    _availableFileList.insert_or_assign(file.name, file.size);
}

QDownloadedFileManager::eDownloadState QDownloadedFileManager::alreadyDownloaded(const sFileInfo& file_)
{
    eDownloadState state = eDownloadState::NOT_DOWNLOADED;

    if (_availableFileList.contains(file_.name) && _availableFileList[file_.name] == file_.size)
    {
        state = eDownloadState::ALREADY_DOWNLOADED_OK;
    }
    else if (_availableFileList.contains(file_.name) && _availableFileList[file_.name] != file_.size)
    {
        state = eDownloadState::ALREADY_DOWNLOADED_SIZE_MISSMATCH;
    }

    return state;
}

QDownloadedFileManager::eDownloadState QDownloadedFileManager::alreadyDownloaded(const std::string& fileName_,
                                                                                 const uint64_t fileSize_)
{
    return this->alreadyDownloaded({.name = fileName_, .size = fileSize_});
}

bool QDownloadedFileManager::getFilePath(IN const std::string& fileName_, OUT std::string& rfilePath_)
{
    bool success = true;

    switch (this->alreadyDownloaded(fileName_, 0u))
    {
        case eDownloadState::NOT_DOWNLOADED:
            success = false;
            break;
        case eDownloadState::ALREADY_DOWNLOADED_SIZE_MISSMATCH:
            [[fallthrough]];
        case eDownloadState::ALREADY_DOWNLOADED_OK:
            success = true;
            break;
    }

    if (success)
    {
        std::string tmpFolderPath = "";
        if (QTmpFolderManager::getInstance().getTmpFolderPath(tmpFolderPath))
        {
            rfilePath_ = tmpFolderPath + "/" + fileName_;
        }
    }

    return success;
}

bool QDownloadedFileManager::getFilePath(INOUT sFileInfo& file_, OUT std::string& rfilePath_)
{
    return this->getFilePath(file_.name, rfilePath_);
}
