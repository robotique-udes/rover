#ifndef __QSSH_WORKER_HPP__
#define __QSSH_WORKER_HPP__

#include "../../Global/Workers/QWorker.hpp"
#include "../QFileItem.hpp"
#include "rovus_lib/macros.h"

class QSshWorker : public QWorker
{
    Q_OBJECT

    static constexpr size_t FILE_DOWNLOAD_BUFFER_SIZE = 4096u;  // 4 kb

  public:
    QSshWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~QSshWorker();

    /**
     * @brief *Async* Retrieves the folder structure at the specified user@host:path and emits newStructureReady() when the new
     * data is ready. Call QSshWorker::getStructure() to retrieve the updated data.
     *
     * @param username_
     * @param hostname_
     * @param path_
     */
    void refreshStructure(std::string username_, std::string hostname_, std::string path_);
    void openFile(IN const std::string& rUsername_, IN const std::string& rHostname_, IN const std::string& rfilePath_);

    std::vector<QFileItem> getFileStructure(void);

  signals:
    /**
     * @brief This signal is used to notify when a structure is ready to be updated by the UI thread
     *
     */
    void newStructureReady(void);

  private:
    void refreshStructureInternal(std::string username_, std::string hostname_, std::string path_);
    void downloadFileInternal(IN const std::string& rUsername_,
                              IN const std::string& rHostname_,
                              IN const std::string& rRemoteFilePath_);
    void openLocalFile(IN const std::string& fileName_);

    std::mutex _filesMutex;
    std::vector<QFileItem> _files;
    static std::mutex _libSshMutex;
};

#endif  // __QSSH_WORKER_HPP__
