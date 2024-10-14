#include "QFileItem.hpp"

QFileItem::QFileItem(std::string name_, std::string extension_, std::string lastModified_)
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

QFileItem::~QFileItem() {}

void QFileItem::addItemToModel(QStandardItemModel& rModel_, bool showHidden_)
{
    if (!showHidden_ && _name->text().size() > 1)
    {
        if (_name->text()[0] == '.' && _name->text()[1] != '.')
        {
            return;
        }
    }

    rModel_.appendRow({_name.get(), _type.get(), _lastModified.get()});
}

std::string QFileItem::getName(void)
{
    return _name->text().toStdString();
}

std::string QFileItem::getType(void)
{
    return _type->text().toStdString();
}

std::string QFileItem::getLastModified(void)
{
    return _lastModified->text().toStdString();
}
