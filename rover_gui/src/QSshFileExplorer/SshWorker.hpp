#ifndef __SSH_WORKER_HPP__
#define __SSH_WORKER_HPP__

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

class SshWorker : public Worker
{
    Q_OBJECT

    static constexpr uint8_t MAX_LOGIN_ATTEMPT = 3u;

  public:
    SshWorker(bool start_ = false, QObject* parent_ = nullptr);
    ~SshWorker();

    void refreshStructure(IN std::string username_, IN std::string hostname_, IN std::string path_);
    std::vector<QFileItem> getStructure(void);

  signals:
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

#endif  // __SSH_WORKER_HPP__
