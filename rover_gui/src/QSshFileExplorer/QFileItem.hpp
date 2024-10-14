#ifndef __Q_FILE_ITEM__
#define __Q_FILE_ITEM__

#include <QStandardItem>

#include "IconManager.hpp"

class QFileItem
{
  public:
    QFileItem(std::string name_, std::string extension_, std::string lastModified_);
    ~QFileItem();

    void addItemToModel(QStandardItemModel& rModel_, bool showHidden_ = false);
    std::string getName(void);
    std::string getType(void);
    std::string getLastModified(void);

  protected:
    std::shared_ptr<QStandardItem> _name = std::make_shared<QStandardItem>();
    std::shared_ptr<QStandardItem> _type = std::make_shared<QStandardItem>();
    std::shared_ptr<QStandardItem> _lastModified = std::make_shared<QStandardItem>();
};

#endif // __Q_FILE_ITEM__
