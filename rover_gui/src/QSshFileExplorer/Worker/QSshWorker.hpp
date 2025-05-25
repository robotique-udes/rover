#ifndef __QSSH_WORKER_HPP__
#define __QSSH_WORKER_HPP__

#include "Global/Workers/QWorker.hpp"
#include "QSshFileExplorer/QFileItem.hpp"
#include "rover_lib2/helpers/macros.hpp"

class QSshFileExplorerWidget;

class QSshWorker : public QWorker
{
    Q_OBJECT

    static constexpr size_t FILE_TRANSFER_BUFFER_SIZE = 1'024'000UL;  // 16 mb/transfer

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
    void refreshStructure(const std::string& username_,
                          const std::string& hostname_,
                          const std::string& oldPath_,
                          const std::string& newPath_);
    void openFile(IN const std::string& rUsername_, IN const std::string& rHostname_, IN const std::string& rfilePath_);
    void transferFile(IN const std::string& fileName_,
                      IN const std::string& ownerUsername_,
                      IN const std::string& ownerHostname_,
                      IN const std::string& ownerFolderPath_,
                      IN const std::string& receiverUsername_,
                      IN const std::string& receiverHostname_,
                      IN const std::string& receiverFolderPath_);

    /**
     * @brief Returns the structure from the last successful retrieval
     *
     * @return std::vector<QFileItem>
     */
    std::vector<QFileItem> getFileStructure(void);

    /**
     * @brief Retrieve the path from the structure returned by getFileStructure()
     *
     * @return std::string
     */
    const std::string& getPathStructure(void);

  signals:
    /**
     * @brief This signal is used to notify when a structure is ready to be updated by the UI thread
     *
     */
    void newStructureReady(void);
    void newProgressBarUpdate(std::string taskDescription_, float progressPercent_);

  private:
    void refreshStructureInternal(const std::string& username_,
                                  const std::string& hostname_,
                                  const std::string& oldPath_,
                                  const std::string& newPath_);
    void downloadFileInternal(IN const std::string& rUsername_,
                              IN const std::string& rHostname_,
                              IN const std::string& rRemoteFilePath_);
    void uploadFileInternal(IN const std::string& rUsername_,
                            IN const std::string& rHostname_,
                            IN const std::string& rRemoteFilePath_,
                            IN const std::string& localFileName_);
    void openLocalFile(IN const std::string& fileName_);

    std::mutex _filesMutex;
    std::vector<QFileItem> _files;
    std::string _filesPath;
    static std::mutex _libSshMutex;
};

#endif  // __QSSH_WORKER_HPP__
