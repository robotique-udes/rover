#ifndef QNAVIGATION_QPATH_QPATHMANAGER
#define QNAVIGATION_QPATH_QPATHMANAGER

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <fstream>

class QPathManager
{
    static constexpr const char* POSITION_FILE_PATH = "/position.csv";

  public:
    QPathManager() {}
    void initializeCSVFile();

    void setSessionFolderPath(std::string sessionFolderPath_);

    void writePosToCSV(double latitude_, double longitude_);

  private:
    std::string _sessionFolderPath;
};

#endif  // QNAVIGATION_QPATH_QPATHMANAGER