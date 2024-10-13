#ifndef __SSH_WORKER_HPP__
#define __SSH_WORKER_HPP__

#include <rclcpp/rclcpp.hpp>

#include <QApplication>
#include <QDateTime>
#include <QMessageBox>
#include <QThread>

#include <libssh/libssh.h>
#include <libssh/sftp.h>

#include "QSshFileExplorer/QFileItem.hpp"
#include "rovus_lib/macros.h"

class SshWorker : public QThread
{
    Q_OBJECT

  private:
    static constexpr uint8_t MAX_LOGIN_ATTEMPT = 3u;

  public:
    SshWorker(QObject* parent_ = nullptr);
    ~SshWorker();
    
    /**
     * @brief *Thread safe* Returns a copy of the current file structure
     * 
     * @return std::vector<QFileItem> 
     */
    std::vector<QFileItem> getStructure(void);

  signals:
    IN void updateStructure(QString rUsername_, QString rHostname_, QString rPath);

    /**
     * @brief Connect to this signal to update the QItemModel when new data is available to be retreived with getStructure()
     * @return OUT
     */
    OUT void newDataReady(void);

  private:
    void run(void) override;
    bool handleAuth(IN const std::string& rUsername_, IN const std::string& rHostname_, OUT ssh_session& pSession_);
    bool askSshSetup(void);
    void sshSetupDialog(const std::string& rUsername_, const std::string& rHostname_);
    void sortAttributeVector(INOUT std::vector<sftp_attributes>& vector);
    std::string getFileExtension(const std::string& filename_);
    std::string unixTimeToString(uint32_t unixTime_);
    
    std::mutex _filesMutex;
    std::vector<QFileItem> _files = {};

  private slots:
    void internalUpdateStructure(QString rUsername_, QString rHostname_, QString rPath);
};

#endif  // __SSH_WORKER_HPP__
