#ifndef __QSSH_WORKER_HPP__
#define __QSSH_WORKER_HPP__

#include <rclcpp/rclcpp.hpp>

#include <QApplication>
#include <QDateTime>
#include <QMessageBox>
#include <QThread>

#include <libssh/libssh.h>
#include <libssh/sftp.h>

#include "../Global/Worker.hpp"
#include "QFileItem.hpp"
#include "rovus_lib/macros.h"

class QSshWorker : public Worker
{
    Q_OBJECT

    static constexpr uint8_t MAX_LOGIN_ATTEMPT = 3u;
    static constexpr uint32_t LOGIN_TIMEOUT = 500'000u;

  public:
    QSshWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~QSshWorker();

    /**
     * @brief Retrieves the folder structure at the specified user@host:path and emits newStructureReady() when the new data is
     * ready
     *
     * @param username_
     * @param hostname_
     * @param path_
     */
    void refreshStructure(IN std::string username_, IN std::string hostname_, IN std::string path_);
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
    bool askSshSetup(void);
    void sshSetupDialog(const std::string& rUsername_, const std::string& rHostname_);
    void sortAttributeVector(INOUT std::vector<sftp_attributes>& vector);
    std::string getFileExtension(const std::string& filename_);
    std::string unixTimeToString(uint32_t unixTime_);

    std::mutex _filesMutex;
    std::vector<QFileItem> _files;
};

#endif  // __QSSH_WORKER_HPP__
