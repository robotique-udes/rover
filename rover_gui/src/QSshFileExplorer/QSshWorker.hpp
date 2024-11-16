#ifndef __QSSH_WORKER_HPP__
#define __QSSH_WORKER_HPP__

#include <libssh/libssh.h>
#include <libssh/sftp.h>

#include "../Global/Workers/QWorker.hpp"
#include "QFileItem.hpp"
#include "rovus_lib/macros.h"

class QSshWorker : public QWorker
{
    Q_OBJECT

    static constexpr uint8_t MAX_LOGIN_ATTEMPT = 3u;
    static constexpr uint32_t LOGIN_TIMEOUT = 5'000'000u;       // us
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

    /**
     * @brief Session needs to be freed with ssh_free after user
     *
     */
    bool getSshSession(IN const std::string& rUsername_, IN const std::string& rHostname_, OUT ssh_session& pSession_);
    bool getSftpSessions(INOUT ssh_session& pSession_, OUT sftp_session& pSftpSession_);

    bool handleSshSetup(const std::string& rUsername_, const std::string& rHostname_) const;
    bool sshSetupDialog(const std::string& rUsername_, const std::string& rHostname_) const;
    void sortAttributeVector(INOUT std::vector<sftp_attributes>& vector_) const;
    std::string getFileExtension(const std::string& filename_) const;
    std::string unixTimeToString(const uint32_t unixTime_) const;
    bool getFileSize(INOUT sftp_session sftp_, const std::string& rFilePath, OUT uint64_t& fileSize_);
    std::string getFileNameFromPath(const std::string& path_);

    std::mutex _filesMutex;
    std::vector<QFileItem> _files;
    static std::mutex _libSshMutex;
};

#endif  // __QSSH_WORKER_HPP__
