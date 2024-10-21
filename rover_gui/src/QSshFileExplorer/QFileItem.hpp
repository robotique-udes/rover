#ifndef __Q_FILE_ITEM__
#define __Q_FILE_ITEM__

#include <QStandardItem>

#include "QIconManager.hpp"

class QFileItem
{
  public:
    QFileItem(const std::string& name_, const std::string& extension_, const std::string& lastModified_);
    ~QFileItem();

    void addItemToModel(QStandardItemModel& rModel_, const bool showHidden_ = false) const;
    std::string getName(void) const;
    std::string getType(void) const;
    std::string getLastModified(void) const;

  protected:
    std::shared_ptr<QStandardItem> _name = std::make_shared<QStandardItem>();
    std::shared_ptr<QStandardItem> _type = std::make_shared<QStandardItem>();
    std::shared_ptr<QStandardItem> _lastModified = std::make_shared<QStandardItem>();
};

#endif  // __Q_FILE_ITEM__
