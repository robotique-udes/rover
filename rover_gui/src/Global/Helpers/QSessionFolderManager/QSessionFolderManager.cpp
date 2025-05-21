#include "QSessionFolderManager.hpp"

QSessionFolderManager::QSessionFolderManager()
{

}

QSessionFolderManager& QSessionFolderManager::getInstance(void)
{
    static QSessionFolderManager instance;
    return instance;
}