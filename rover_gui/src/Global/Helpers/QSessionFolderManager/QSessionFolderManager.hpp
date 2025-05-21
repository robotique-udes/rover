#include <string>

class QSessionFolderManager
{
    public:
    static QSessionFolderManager& getInstance(void);

    private:
    QSessionFolderManager();
    ~QSessionFolderManager() = default;

    QSessionFolderManager(const QSessionFolderManager&) = delete;
    QSessionFolderManager& operator=(const QSessionFolderManager&) = delete;

    std::string _sessionFolderPath;
};