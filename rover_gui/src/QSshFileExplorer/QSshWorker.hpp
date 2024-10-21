#ifndef __QSSH_WORKER_HPP__
#define __QSSH_WORKER_HPP__

#include <libssh/libssh.h>
#include <libssh/sftp.h>

#include "../Global/QWorker.hpp"
#include "QFileItem.hpp"
#include "rovus_lib/macros.h"

class QSshWorker : public QWorker
{
    Q_OBJECT

    static constexpr uint8_t MAX_LOGIN_ATTEMPT = 3u;
    static constexpr uint32_t LOGIN_TIMEOUT = 500'000u;

  public:
    QSshWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~QSshWorker();

    /**
     * @brief *Async* Retrieves the folder structure at the specified user@host:path and emits newStructureReady() when the new
     * data is ready
     *
     * @param username_
     * @param hostname_
     * @param path_
     */
    void refreshStructure(std::string username_, std::string hostname_, std::string path_);
    std::vector<QFileItem> getStructure(void);

  signals:
    /**
     * @brief This signal is used to notify when a structure is ready to be updated by the UI thread
     *
     */
    void newStructureReady(void);

  private:
    void refreshStructureInternal(std::string username_, std::string hostname_, std::string path_);
    bool handleAuth(IN const std::string& rUsername_, IN const std::string& rHostname_, OUT ssh_session& pSession_);
    bool askSshSetup(void) const;
    void sshSetupDialog(const std::string& rUsername_, const std::string& rHostname_) const;
    void sortAttributeVector(INOUT std::vector<sftp_attributes>& vector_) const;
    std::string getFileExtension(const std::string& filename_) const;
    std::string unixTimeToString(const uint32_t unixTime_) const;

    std::mutex _filesMutex;
    std::vector<QFileItem> _files;
};

#endif  // __QSSH_WORKER_HPP__
