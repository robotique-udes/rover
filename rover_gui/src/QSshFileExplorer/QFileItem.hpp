#ifndef __Q_FILE_ITEM__
#define __Q_FILE_ITEM__

#include <QStandardItem>
#include <QStandardItemModel>
#include <rclcpp/rclcpp.hpp>

#include "IconManager.hpp"

class QFileItem
{
  public:
    QFileItem(std::string name_, std::string extension_, std::string lastModified_)
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

    void addItemToModel(QStandardItemModel& rModel_, bool showHidden_ = false)
    {
        if (!showHidden_ && _name->text().size() > 1)
        {
            if (_name->text()[0] == '.' && _name->text()[1] != '.' )
            {
                return;
            }
        }

        rModel_.appendRow({_name.get(), _type.get(), _lastModified.get()});
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
    std::shared_ptr<QStandardItem> _name = std::make_shared<QStandardItem>();
    std::shared_ptr<QStandardItem> _type = std::make_shared<QStandardItem>();
    std::shared_ptr<QStandardItem> _lastModified = std::make_shared<QStandardItem>();
};

#endif // __Q_FILE_ITEM__
