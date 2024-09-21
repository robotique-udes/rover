#pragma once

#include <rclcpp/rclcpp.hpp>
#include <QIcon>

class IconManager
{
public:
    static std::map<std::string, QIcon> &getIconMap()
    {
        if (!_isInit)
        {
            IconManager::initializeIcons();
            _isInit = true;
        }
        return _iconMap;
    }

private:
    void static initializeIcons();

    static bool _isInit;
    static std::map<std::string, QIcon> _iconMap;
};

bool IconManager::_isInit = false;
std::map<std::string, QIcon> IconManager::_iconMap;

void IconManager::initializeIcons()
{
    IconManager::_iconMap[""] = QIcon::fromTheme("folder");
    IconManager::_iconMap[".goBack"] = QIcon::fromTheme("go-previous");
    IconManager::_iconMap[".txt"] = QIcon::fromTheme("text-x-generic");
    IconManager::_iconMap[".md"] = QIcon::fromTheme("text-x-generic");
    IconManager::_iconMap[".rtf"] = QIcon::fromTheme("text-x-generic");
    IconManager::_iconMap[".png"] = QIcon::fromTheme("image-x-generic");
    IconManager::_iconMap[".jpg"] = QIcon::fromTheme("image-x-generic");
    IconManager::_iconMap[".jpeg"] = QIcon::fromTheme("image-x-generic");
    IconManager::_iconMap[".bmp"] = QIcon::fromTheme("image-x-generic");
    IconManager::_iconMap[".gif"] = QIcon::fromTheme("image-x-generic");
    IconManager::_iconMap[".pdf"] = QIcon::fromTheme("application-pdf");
    IconManager::_iconMap[".doc"] = QIcon::fromTheme("application-vnd.openxmlformats-officedocument.wordprocessingml.document");
    IconManager::_iconMap[".docx"] = QIcon::fromTheme("application-vnd.openxmlformats-officedocument.wordprocessingml.document");
    IconManager::_iconMap[".odt"] = QIcon::fromTheme("application-vnd.oasis.opendocument.text");
    IconManager::_iconMap[".cpp"] = QIcon::fromTheme("text-x-c++src");
    IconManager::_iconMap[".hpp"] = QIcon::fromTheme("text-x-c++src");
    IconManager::_iconMap[".h"] = QIcon::fromTheme("text-x-c++src");
    IconManager::_iconMap[".py"] = QIcon::fromTheme("text-x-python");
    IconManager::_iconMap[".java"] = QIcon::fromTheme("text-x-java");
    IconManager::_iconMap[".js"] = QIcon::fromTheme("text-x-script");
    IconManager::_iconMap[".html"] = QIcon::fromTheme("text-html");
    IconManager::_iconMap[".css"] = QIcon::fromTheme("text-css");
    IconManager::_iconMap[".zip"] = QIcon::fromTheme("application-zip");
    IconManager::_iconMap[".rar"] = QIcon::fromTheme("application-x-rar");
    IconManager::_iconMap[".7z"] = QIcon::fromTheme("application-x-7z-compressed");
    IconManager::_iconMap[".tar"] = QIcon::fromTheme("application-x-tar");
    IconManager::_iconMap[".gz"] = QIcon::fromTheme("application-x-gzip");
    IconManager::_iconMap[".mp3"] = QIcon::fromTheme("audio-x-mp3");
    IconManager::_iconMap[".wav"] = QIcon::fromTheme("audio-x-wav");
    IconManager::_iconMap[".flac"] = QIcon::fromTheme("audio-x-flac");
    IconManager::_iconMap[".mp4"] = QIcon::fromTheme("video-x-mp4");
    IconManager::_iconMap[".avi"] = QIcon::fromTheme("video-x-msvideo");
    IconManager::_iconMap[".mkv"] = QIcon::fromTheme("video-x-matroska");
    IconManager::_iconMap[".mov"] = QIcon::fromTheme("video-x-quicktime");
}
