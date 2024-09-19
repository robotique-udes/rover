#pragma once

#include <QApplication>
#include <QStandardItem>
#include <QStandardItemModel>
#include "rclcpp/rclcpp.hpp"

class IconManager
{
public:
    static std::map<std::string, QIcon> &getIconMap()
    {
        static std::map<std::string, QIcon> iconMap;
        return iconMap;
    }

    static void initializeIcons()
    {
        auto &iconMap = getIconMap();
        iconMap[""] = QIcon::fromTheme("folder");
        iconMap[".goBack"] = QIcon::fromTheme("go-back");
        iconMap[".txt"] = QIcon::fromTheme("text-x-generic");
        iconMap[".md"] = QIcon::fromTheme("text-x-generic");
        iconMap[".rtf"] = QIcon::fromTheme("text-x-generic");
        iconMap[".png"] = QIcon::fromTheme("image-x-generic");
        iconMap[".jpg"] = QIcon::fromTheme("image-x-generic");
        iconMap[".jpeg"] = QIcon::fromTheme("image-x-generic");
        iconMap[".bmp"] = QIcon::fromTheme("image-x-generic");
        iconMap[".gif"] = QIcon::fromTheme("image-x-generic");
        iconMap[".pdf"] = QIcon::fromTheme("application-pdf");
        iconMap[".doc"] = QIcon::fromTheme("application-vnd.openxmlformats-officedocument.wordprocessingml.document");
        iconMap[".docx"] = QIcon::fromTheme("application-vnd.openxmlformats-officedocument.wordprocessingml.document");
        iconMap[".odt"] = QIcon::fromTheme("application-vnd.oasis.opendocument.text");
        iconMap[".cpp"] = QIcon::fromTheme("text-x-c++src");
        iconMap[".hpp"] = QIcon::fromTheme("text-x-c++src");
        iconMap[".h"] = QIcon::fromTheme("text-x-c++src");
        iconMap[".py"] = QIcon::fromTheme("text-x-python");
        iconMap[".java"] = QIcon::fromTheme("text-x-java");
        iconMap[".js"] = QIcon::fromTheme("text-x-script");
        iconMap[".html"] = QIcon::fromTheme("text-html");
        iconMap[".css"] = QIcon::fromTheme("text-css");
        iconMap[".zip"] = QIcon::fromTheme("application-zip");
        iconMap[".rar"] = QIcon::fromTheme("application-x-rar");
        iconMap[".7z"] = QIcon::fromTheme("application-x-7z-compressed");
        iconMap[".tar"] = QIcon::fromTheme("application-x-tar");
        iconMap[".gz"] = QIcon::fromTheme("application-x-gzip");
        iconMap[".mp3"] = QIcon::fromTheme("audio-x-mp3");
        iconMap[".wav"] = QIcon::fromTheme("audio-x-wav");
        iconMap[".flac"] = QIcon::fromTheme("audio-x-flac");
        iconMap[".mp4"] = QIcon::fromTheme("video-x-mp4");
        iconMap[".avi"] = QIcon::fromTheme("video-x-msvideo");
        iconMap[".mkv"] = QIcon::fromTheme("video-x-matroska");
        iconMap[".mov"] = QIcon::fromTheme("video-x-quicktime");
    }
};

class QFileItem
{
private:
    enum class eFileType : uint8_t
    {
    };

public:
    QFileItem(std::string name_, std::string extension_, std::string lastModified_) : _name(new QStandardItem), _type(new QStandardItem), _lastModified(new QStandardItem)
    {
        _name->setText(QString(name_.c_str()));
        _name->setCheckable(false);
        _name->setEditable(false);

        std::map<std::string, QIcon>::const_iterator it = IconManager::getIconMap().find(extension_);
        if (it != IconManager::getIconMap().end())
        {
            _name->setIcon(it->second);
        }
        else
        {
            _name->setIcon(QIcon::fromTheme("text-x-generic"));
        }

        _type->setCheckable(false);
        _type->setEditable(false);
        _type->setText(QString(extension_.c_str()));

        _lastModified->setCheckable(false);
        _lastModified->setEditable(false);
        _lastModified->setText(QString(lastModified_.c_str()));
    }

    void addItemToModel(QStandardItemModel &rModel_)
    {
        rModel_.appendRow({_name.release(), _type.release(), _lastModified.release()});
    }

    std::string getName(void)
    {
        return _name->text().toStdString();
    }

    std::string getType(void)
    {
        return _type->text().toStdString();
    }

    std::string getLastModified(void)
    {
        return _lastModified->text().toStdString();
    }

protected:
    std::unique_ptr<QStandardItem> _name;
    std::unique_ptr<QStandardItem> _type;
    std::unique_ptr<QStandardItem> _lastModified;
};

// Special items
class QFileItemGoBack : public QFileItem
{
public:
    QFileItemGoBack() : QFileItem(". . .", ".goBack", "")
    {
        _type->setText("");
    }
};
