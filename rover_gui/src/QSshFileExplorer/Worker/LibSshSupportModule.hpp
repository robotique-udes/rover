#include <rclcpp/rclcpp.hpp>

#include "rovus_lib/macros.h"
#include <libssh/libssh.h>
#include <libssh/sftp.h>

namespace LibSshSupportModule
{
    constexpr uint8_t MAX_LOGIN_ATTEMPT = 3u;
    constexpr uint32_t LOGIN_TIMEOUT = 5'000'000u;  // us

    bool getSshSession(IN const std::string& rUsername_, IN const std::string& rHostname_, OUT ssh_session& pSession_);
    bool getSftpSessions(INOUT ssh_session& pSession_, OUT sftp_session& pSftpSession_);

    bool SshNoConnectionDialog(const std::string& rUsername_, const std::string& rHostname_);
    bool sshSetupDialog(const std::string& rUsername_, const std::string& rHostname_);
    void sortSftpAttributeVector(INOUT std::vector<sftp_attributes>& vector_);
    std::string getFileExtension(const std::string& filename_);
    std::string unixTimeToString(const uint32_t unixTime_);
    bool getSftpFileSize(INOUT sftp_session sftp_, const std::string& rFilePath, OUT uint64_t& fileSize_);
}  // namespace LibSshSupportModule
