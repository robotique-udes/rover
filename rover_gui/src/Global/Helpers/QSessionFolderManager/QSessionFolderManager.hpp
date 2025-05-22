#include "rover_lib2/helpers/macros.hpp"

#include <string>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>

class QSessionFolderManager
{
    public:
    static QSessionFolderManager& getInstance(void);
    bool getSessionFolderPath(OUT std::string& path_) const;
    
    private:
    QSessionFolderManager();
    ~QSessionFolderManager() = default;

    QSessionFolderManager(const QSessionFolderManager&) = delete;
    QSessionFolderManager& operator=(const QSessionFolderManager&) = delete;

    std::string getCurrentTime(void);
    bool createDirectory(const std::string& path_) const;

    bool _valid;
    std::string _sessionFolderPath;
};