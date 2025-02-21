#ifndef __QTMP_FOLDER_MANAGER__
#define __QTMP_FOLDER_MANAGER__

#include "rovus_lib/macros.h"

#include <string>

/**
 * @brief Singleton handle to manage temporary files and folders, existing old
 * sessions are deleted at construction but current session files/folders are
 *  not deleted at exit to keep a "backup"
 *
 */
class QTmpFolderManager
{
  public:
    /**
     * @brief Return a valid reference to singleton instance. Creates it if not
     * already existing.
     *
     * @return QTmpFolderManager&
     */
    static QTmpFolderManager& getInstance();

    /**
     * @brief Return the current tmp folder path for the current session
     *
     * @param path_ return the path of the shared tmp folder on success
     * @return success
     */
    bool getTmpFolderPath(OUT std::string& path_) const;

    /**
     * @brief Creates and returns a unique new temporary folder to handle name
     * collisions. Cannot be cleaned by this object in the current session, it's
     * the user's job to keep the reference.
     *
     * @param path_ return the path of the new unique shared tmp folder on
     *  success
     * @return success
     */
    bool getUniqueTmpFolderPath(OUT std::string& path_) const;

  private:
    QTmpFolderManager();
    ~QTmpFolderManager() = default;

    QTmpFolderManager(const QTmpFolderManager&) = delete;
    QTmpFolderManager& operator=(const QTmpFolderManager&) = delete;

    bool createDirectory(const std::string& path_) const;

    bool _valid;
    std::string _tempFolderPath;
};

#endif  // __QTMP_FOLDER_MANAGER__
