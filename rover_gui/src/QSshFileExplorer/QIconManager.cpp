#include "QIconManager.hpp"

const QIconManager& QIconManager::getInstance(void)
{
    static QIconManager instance;
    return instance;
}

const QIcon& QIconManager::getIcon(const std::string& extension_) const
{
    if (_iconMap.find(extension_) != _iconMap.end())
    {
        return _iconMap.find(extension_)->second;
    }
    else
    {
        return _iconMap.find("txt")->second;
    }
}

QIconManager::QIconManager()
{
    initializeIcons();
}

void QIconManager::initializeIcons()
{
    _iconMap[""] = QIcon::fromTheme("folder");
    _iconMap["goBack"] = QIcon::fromTheme("go-previous");
    _iconMap["txt"] = QIcon::fromTheme("text-x-generic");
    _iconMap["md"] = QIcon::fromTheme("text-x-generic");
    _iconMap["rtf"] = QIcon::fromTheme("text-x-generic");
    _iconMap["png"] = QIcon::fromTheme("image-x-generic");
    _iconMap["jpg"] = QIcon::fromTheme("image-x-generic");
    _iconMap["jpeg"] = QIcon::fromTheme("image-x-generic");
    _iconMap["bmp"] = QIcon::fromTheme("image-x-generic");
    _iconMap["gif"] = QIcon::fromTheme("image-x-generic");
    _iconMap["pdf"] = QIcon::fromTheme("application-pdf");
    _iconMap["doc"] = QIcon::fromTheme("application-vnd.openxmlformats-officedocument.wordprocessingml.document");
    _iconMap["docx"] = QIcon::fromTheme("application-vnd.openxmlformats-officedocument.wordprocessingml.document");
    _iconMap["odt"] = QIcon::fromTheme("application-vnd.oasis.opendocument.text");
    _iconMap["cpp"] = QIcon::fromTheme("text-x-c++src");
    _iconMap["hpp"] = QIcon::fromTheme("text-x-c++src");
    _iconMap["h"] = QIcon::fromTheme("text-x-c++src");
    _iconMap["py"] = QIcon::fromTheme("text-x-python");
    _iconMap["java"] = QIcon::fromTheme("text-x-java");
    _iconMap["js"] = QIcon::fromTheme("text-x-script");
    _iconMap["html"] = QIcon::fromTheme("text-html");
    _iconMap["css"] = QIcon::fromTheme("text-css");
    _iconMap["zip"] = QIcon::fromTheme("application-zip");
    _iconMap["rar"] = QIcon::fromTheme("application-x-rar");
    _iconMap["7z"] = QIcon::fromTheme("application-x-7z-compressed");
    _iconMap["tar"] = QIcon::fromTheme("application-x-tar");
    _iconMap["gz"] = QIcon::fromTheme("application-x-gzip");
    _iconMap["mp3"] = QIcon::fromTheme("audio-x-mp3");
    _iconMap["wav"] = QIcon::fromTheme("audio-x-wav");
    _iconMap["flac"] = QIcon::fromTheme("audio-x-flac");
    _iconMap["mp4"] = QIcon::fromTheme("video-x-mp4");
    _iconMap["avi"] = QIcon::fromTheme("video-x-msvideo");
    _iconMap["mkv"] = QIcon::fromTheme("video-x-matroska");
    _iconMap["mov"] = QIcon::fromTheme("video-x-quicktime");
}
