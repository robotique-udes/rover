#pragma once

#include <QStandardItem>
#include <QStandardItemModel>
#include <rclcpp/rclcpp.hpp>

#include "icon_manager.hpp"

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
    QFileItemGoBack() : QFileItem("...", ".goBack", "")
    {
        _type->setText("");
    }
};
