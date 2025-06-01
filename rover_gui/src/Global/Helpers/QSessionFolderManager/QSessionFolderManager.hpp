#include "rover_lib2/helpers/macros.hpp"
#include <array>
#include <cstdlib>
#include <string>
#include <optional>

class QSessionFolderManager
{
  private:
    static constexpr std::array<const char*, 1> SUBDIRECTORIES = {"/camera"};

  public:
    static QSessionFolderManager& getInstance(void);
    std::optional<std::string> getSessionFolderPath(void) const;

  private:
    QSessionFolderManager();
    ~QSessionFolderManager() = default;

    QSessionFolderManager(const QSessionFolderManager&) = delete;
    QSessionFolderManager& operator=(const QSessionFolderManager&) = delete;

    std::string _sessionFolderPath;
};