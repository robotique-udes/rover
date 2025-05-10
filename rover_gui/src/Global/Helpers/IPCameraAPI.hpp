#pragma once

#define slots Q_SLOTS  // Conflict with pybind11
#undef slots          
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>

#include <string>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <chrono>

#define slots Q_SLOTS 
namespace py = pybind11;

// Forward declaration
class PyObjectWrapper;

// Enum classes mirroring the Python enums
namespace camera {

// Mirror of Python's Scenes enum
enum class Scenes {
    OUTDOOR = 0,
    INDOOR = 2
};

// Mirror of Python's ExposureModes enum
enum class ExposureModes {
    SCENE = 0,
    MANUAL = 1,
    SHUTTER = 2
};

// Mirror of Python's WhiteBalanceModes enum
enum class WhiteBalanceModes {
    AUTO = 0,
    MANUAL = 1,
    INDOOR = 8,
    OUTDOOR = 9,
    SUNLIGHT = 2
};

// Mirror of Python's IRModes enum
enum class IRModes {
    AUTO = 4,
    DAY = 3,
    NIGHT = 2
};

// Mirror of Python's FramerateValues enum
enum class FramerateValues {
    _3 = 3,
    _5 = 5,
    _10 = 10,
    _15 = 15,
    _20 = 20,
    _25 = 25,
    _30 = 30
};

// Mirror of Python's ShutterValues enum
enum class ShutterValues {
    _1_8000 = 0,
    _1_6000 = 1,
    _1_4000 = 2,
    _1_2000 = 3,
    _1_1000 = 4,
    _1_500 = 5,
    _1_250 = 6,
    _1_200 = 7,
    _1_150 = 8,
    _1_100 = 9,
    _1_50 = 10,
    _1_25 = 11,
    _1_20 = 12,
    _1_15 = 13,
    _1_10 = 14,
    _1_8 = 15,
    _1_5 = 16,
    _1_3 = 17,
    _1_2 = 18,
    _1 = 19
};

// Mirror of Python's AEGains enum
enum class AEGains {
    _1X = 0,
    _2X = 1,
    _4X = 2,
    _8X = 3,
    _16X = 4,
    _32X = 5,
    _64X = 6
};

} // namespace camera

// Use the same visibility attribute as pybind11 to avoid warnings
class PYBIND11_EXPORT ParameterHandler {
private:
    // Connection parameters
    int default_port;
    std::string default_username;
    std::string default_password;
    
    // Map of IP addresses to Python controller objects
    struct CameraConnection {
        std::unique_ptr<PyObjectWrapper> controller;
        bool is_connected = false;
        std::chrono::system_clock::time_point last_check_time;
    };
    
    std::unordered_map<std::string, CameraConnection> connectionCache;
    std::mutex cacheMutex;
    
    // Python module reference 
    py::object ipcamera_api_module;
    
    // Get or create a controller for the given IP
    PyObjectWrapper* getOrCreateController(const std::string& ip);
    
    // Helper to get Python enum instances
    py::object getPythonEnum(const std::string& enum_class, int value);

public:
    // Constructor with connection parameters
    explicit ParameterHandler(
        const std::string& username = "admin", 
        const std::string& password = "admin",
        int port = 8999
    );
    
    // Destructor
    ~ParameterHandler();
    
    // Set Python module path
    static void setPythonModulePath(const std::string& path);

    // Check if camera is reachable
    bool isCameraReachable(const std::string& ip, bool force_check = false);

    // Camera parameter control methods - each takes an IP address
    bool setBrightness(const std::string& ip, int value);
    bool setContrast(const std::string& ip, int value);
    bool setSaturation(const std::string& ip, int value);
    bool setSharpness(const std::string& ip, int value);
    bool setResolution(const std::string& ip, const std::string& value);
    bool setFrameRate(const std::string& ip, int value);
    bool setBitrate(const std::string& ip, int value);
    
    bool disableWideDynamicRange(const std::string& ip);
    bool setWideDynamicRangeLevel(const std::string& ip, int value);
    
    bool enableBackLight(const std::string& ip);
    bool disableBackLight(const std::string& ip);
    
    bool HorizontalMirror(const std::string& ip);
    bool VerticalMirror(const std::string& ip);
    bool resetHorizontalMirror(const std::string& ip);
    bool resetVerticalMirror(const std::string& ip);
    
    bool enableAntiFalseColor(const std::string& ip);
    bool disableAntiFalseColor(const std::string& ip);
    
    bool enableDigitalImageStabilizer(const std::string& ip);
    bool disableDigitalImageStabilizer(const std::string& ip);
    
    bool enableLensShadeCorrection(const std::string& ip);
    bool disableLensShadeCorrection(const std::string& ip);
    
    bool setLensDistortionCorrection(const std::string& ip, int value);
    bool disableLensDistortionCorrection(const std::string& ip);
    
    bool setAntiFog(const std::string& ip, int value);
    bool disableAntiFog(const std::string& ip);
    
    bool setScene(const std::string& ip, int sceneValue);
    bool setExposureMode(const std::string& ip, int modeValue);
    bool setShutterSpeed(const std::string& ip, int valueNum);
    bool setManualACG(const std::string& ip, int valueNum);
    bool setWhiteBalanceMode(const std::string& ip, int modeValue);
    bool setIRMode(const std::string& ip, int modeValue);
    bool disableIR(const std::string& ip);
};