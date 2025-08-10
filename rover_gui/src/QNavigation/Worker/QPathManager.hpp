#ifndef QNAVIGATION_QPATH_QPATHMANAGER
#define QNAVIGATION_QPATH_QPATHMANAGER

#include <rclcpp/rclcpp.hpp>
#include "Global/Workers/QWorker.hpp"

#include <iostream>
#include <fstream>
#include <vector>

class QPathManager : public QWorker
{
    Q_OBJECT

    static constexpr const char* NAVIGATION_PATH = "/Navigation";
    static constexpr const char* POSITION_FILE_PATH = "/position.csv";

  signals:
    void loadOldPath(double latitude_, double longitude_);

  public:
    QPathManager(bool start_ = false, QObject* parent_ = nullptr);

    void setSessionFolderPath(std::string sessionFolderPath_);
    void initializeCSVFile(void);
    void writePosToCSV(double latitude_, double longitude_);

  private:
    void writePosToCSVInternal(double latitude_, double longitude_);
    std::string findLastSessionFolder(void);
    void readFromCSV(std::string filePath_);

    std::string _sessionFolderPath;
};

#endif  // QNAVIGATION_QPATH_QPATHMANAGER