#ifndef __QTMP_FOLDER_MANAGER__
#define __QTMP_FOLDER_MANAGER__

#include <QString>
#include <string>
#include "rovus_lib/macros.h"

class QTmpFolderManager
{
  public:
    static QTmpFolderManager& getInstance();

    bool getTmpFolderPath(OUT std::string& path_) const;
    bool getUniqueTmpFolderPath(OUT std::string& path_) const;

  private:
    QTmpFolderManager();
    ~QTmpFolderManager() = default;

    QTmpFolderManager(const QTmpFolderManager&) = delete;
    QTmpFolderManager& operator=(const QTmpFolderManager&) = delete;

    bool createDirectory(IN const std::string& path_) const;

    bool _valid;
    std::string _tempFolderPath;
};

#endif  // __QTMP_FOLDER_MANAGER__
