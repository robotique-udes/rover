#pragma once

#include <QStandardItem>
#include <QApplication>

class Item
{
private:
enum class eFileType: uint8_t
{
};

public:

    Item(std::string name_, std::string extension_, std::string lastModified_) :
    _name(new QStandardItem), _type(new QStandardItem), _lastModified(new QStandardItem)
    {
        _name->setText(QString(name_.c_str()));
        _name->setCheckable(false);
        _name->setEditable(false);
        // _name->getRelativeIcon(extension_)

        _type->setCheckable(false);
        _type->setEditable(false);
        _type->setText("Folder");

        _lastModified->setCheckable(false);
        _lastModified->setEditable(false);
        _lastModified->setText("2024-09-17");
        _lastModified->setTextAlignment(Qt::AlignRight);
    }

private:
    QStandardItem* _name;
    QStandardItem* _type;
    QStandardItem* _lastModified;

    // eFileType getFileType(std::string fileExtension_){}
};
