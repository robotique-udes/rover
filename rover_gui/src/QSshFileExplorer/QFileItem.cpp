#include "QFileItem.hpp"

QFileItem::QFileItem(const std::string& name_, const std::string& extension_, const std::string& lastModified_)
{
    _name->setText(QString(name_.c_str()));
    _name->setCheckable(false);
    _name->setEditable(false);

    _name->setIcon(QIconManager::getInstance().getIcon(extension_));

    _type->setCheckable(false);
    _type->setEditable(false);
    _type->setText(QString(extension_.c_str()));

    _lastModified->setCheckable(false);
    _lastModified->setEditable(false);
    _lastModified->setText(QString(lastModified_.c_str()));
}

QFileItem::~QFileItem() {}

void QFileItem::addItemToModel(QStandardItemModel& rModel_, const bool showHidden_) const
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

std::string QFileItem::getName(void) const
{
    return _name->text().toStdString();
}

std::string QFileItem::getType(void) const
{
    return _type->text().toStdString();
}

std::string QFileItem::getLastModified(void) const
{
    return _lastModified->text().toStdString();
}
