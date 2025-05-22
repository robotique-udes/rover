#include <string>
#include <cstdlib>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <Global/Helpers/QToastNotification/QToastNotification.hpp>

class QSessionFolderManager
{
    public:
    static QSessionFolderManager& getInstance(void);

    private:
    QSessionFolderManager();
    ~QSessionFolderManager() = default;

    QSessionFolderManager(const QSessionFolderManager&) = delete;
    QSessionFolderManager& operator=(const QSessionFolderManager&) = delete;

    std::string getCurrentTime(void);

    std::string _sessionFolderPath;
};