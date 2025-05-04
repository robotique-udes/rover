// In IPCameraAPI.cpp
#include "Global/Helpers/IPCameraAPI.hpp"
#include <pybind11/embed.h>
#include <iostream>
#include <filesystem>

namespace py = pybind11;
namespace fs = std::filesystem;

// Simple wrapper for Python objects - add same visibility attribute as py::object
class PYBIND11_EXPORT PyObjectWrapper {
public:
    py::object obj;
    
    PyObjectWrapper(const py::object& o) : obj(o) {}
    ~PyObjectWrapper() = default;
};

static bool pyInitialized = false;

ParameterHandler::ParameterHandler(
    const std::string& username, 
    const std::string& password,
    int port
) : default_port(port),
    default_username(username),
    default_password(password)
{

    if (!pyInitialized) {
        py::initialize_interpreter();
        pyInitialized = true;
        
        try {
            py::module sys = py::module::import("sys");
            py::list py_path = sys.attr("path").cast<py::list>();
            
            const char* home_dir = std::getenv("HOME");
            if (home_dir != nullptr) {
                std::string ros2_path = std::string(home_dir) + "/ros2_ws/build/rover_gui/python_modules";
                
                py_path.append(ros2_path);
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Error setting Python path: " << e.what() << std::endl;
        }
    }
    
    try {
        ipcamera_api_module = py::module::import("ipcamera_api");
    }
    catch (const py::error_already_set& e) {
        std::cerr << "Failed to import ipcamera_api module: " << e.what() << std::endl;
    }
}

// Destructor implementation
ParameterHandler::~ParameterHandler() {
    // Clear all camera connections
    connectionCache.clear();
}

void ParameterHandler::setPythonModulePath(const std::string& path) {
    if (!pyInitialized) {
        py::initialize_interpreter();
        pyInitialized = true;
    }

    try {
        py::module sys = py::module::import("sys");
        py::list py_path = sys.attr("path").cast<py::list>();
        
        // Check if path already exists in sys.path
        bool path_exists = false;
        for (const auto& p : py_path) {
            if (std::string(py::str(p)) == path) {
                path_exists = true;
                break;
            }
        }
        
        if (!path_exists) {
            py_path.append(path);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error setting Python path: " << e.what() << std::endl;
    }
}

PyObjectWrapper* ParameterHandler::getOrCreateController(const std::string& ip) {
    // Check if the Python module was successfully loaded
    if (ipcamera_api_module.is_none()) {
        std::cerr << "Python module was not loaded. Cannot create controller for " << ip << std::endl;
        return nullptr;
    }
    
    // Thread safety for cache access
    std::lock_guard<std::mutex> lock(cacheMutex);
    
    // Check if we have a cached connection
    auto it = connectionCache.find(ip);
    if (it != connectionCache.end() && it->second.controller != nullptr) {
        return it->second.controller.get();
    }
    
    // Create a new connection
    try {
        // Create an instance of the Python class
        py::object controller = ipcamera_api_module.attr("ParameterHandler")(
            ip, default_port, default_username, default_password
        );
        
        // Store the Python controller instance in our cache
        CameraConnection conn;
        conn.controller = std::make_unique<PyObjectWrapper>(controller);
        
        connectionCache[ip] = std::move(conn);
        
        return connectionCache[ip].controller.get();
    }
    catch (const py::error_already_set& e) {
        std::cerr << "Python error connecting to camera at " << ip << ": " << e.what() << std::endl;
        return nullptr;
    }
    catch (const std::exception& e) {
        std::cerr << "C++ error connecting to camera at " << ip << ": " << e.what() << std::endl;
        return nullptr;
    }
}

py::object ParameterHandler::getPythonEnum(const std::string& enum_class, int value) {
    try {
        return ipcamera_api_module.attr(enum_class.c_str())(value);
    }
    catch (const py::error_already_set& e) {
        std::cerr << "Python error getting " << enum_class << " enum value " << value << ": " << e.what() << std::endl;
        throw;
    }
}

// Helper template function to call Python methods
template<typename... Args>
bool callPythonMethod(PyObjectWrapper* wrapper, const std::string& ip, const char* methodName, Args&&... args) {
    if (!wrapper) {
        std::cerr << "Failed to get controller for camera at " << ip << std::endl;
        return false;
    }
    
    try {
        py::object result = wrapper->obj.attr(methodName)(std::forward<Args>(args)...);
        return py::cast<bool>(result);
    }
    catch (const py::error_already_set& e) {
        std::cerr << "Python error calling " << methodName << " on camera " << ip << ": " << e.what() << std::endl;
        return false;
    }
    catch (const std::exception& e) {
        std::cerr << "C++ error calling " << methodName << " on camera " << ip << ": " << e.what() << std::endl;
        return false;
    }
}

// Camera parameter control methods implementation
bool ParameterHandler::setBrightness(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "setBrightness", value);
}

bool ParameterHandler::setContrast(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "setContrast", value);
}

bool ParameterHandler::setSaturation(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "setSaturation", value);
}

bool ParameterHandler::setSharpness(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "setSharpness", value);
}

bool ParameterHandler::setResolution(const std::string& ip, const std::string& value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "setResolution", value);
}

bool ParameterHandler::setFrameRate(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object frameRateEnum = getPythonEnum("FramerateValues", value);
        return callPythonMethod(controller, ip, "setFrameRate", frameRateEnum);
    }
    catch (const std::exception& e) {
        std::cerr << "Error with FramerateValues enum for " << ip << ": " << e.what() << std::endl;
        return false;
    }
}

bool ParameterHandler::setBitrate(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "setBitrate", value);
}

bool ParameterHandler::disableWideDynamicRange(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "disableWideDynamicRange");
}

bool ParameterHandler::setWideDynamicRangeLevel(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "setWideDynamicRangeLevel", value);
}

bool ParameterHandler::enableBackLight(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "enableBackLight");
}

bool ParameterHandler::disableBackLight(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "disableBackLight");
}

bool ParameterHandler::HorizontalMirror(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "HorizontalMirror");
}

bool ParameterHandler::VerticalMirror(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "VerticalMirror");
}

bool ParameterHandler::resetHorizontalMirror(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "resetHorizontalMirror");
}

bool ParameterHandler::resetVerticalMirror(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "resetVerticalMirror");
}

bool ParameterHandler::enableAntiFalseColor(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "enableAntiFalseColor");
}

bool ParameterHandler::disableAntiFalseColor(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "disableAntiFalseColor");
}

bool ParameterHandler::enableDigitalImageStabilizer(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "enableDigitalImageStabilizer");
}

bool ParameterHandler::disableDigitalImageStabilizer(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "disableDigitalImageStabilizer");
}

bool ParameterHandler::enableLensShadeCorrection(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "enableLensShadeCorrection");
}

bool ParameterHandler::disableLensShadeCorrection(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "disableLensShadeCorrection");
}

bool ParameterHandler::setLensDistortionCorrection(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "setLensDistortionCorrection", value);
}

bool ParameterHandler::disableLensDistortionCorrection(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "disableLensDistortionCorrection");
}

bool ParameterHandler::setAntiFog(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "setAntiFog", value);
}

bool ParameterHandler::disableAntiFog(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "disableAntiFog");
}

bool ParameterHandler::setScene(const std::string& ip, int sceneValue) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object scenesEnum = getPythonEnum("Scenes", sceneValue);
        return callPythonMethod(controller, ip, "setScene", scenesEnum);
    }
    catch (const std::exception& e) {
        std::cerr << "Error with Scenes enum for " << ip << ": " << e.what() << std::endl;
        return false;
    }
}

bool ParameterHandler::setExposureMode(const std::string& ip, int modeValue) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object exposureModeEnum = getPythonEnum("ExposureModes", modeValue);
        return callPythonMethod(controller, ip, "setExposureMode", exposureModeEnum);
    }
    catch (const std::exception& e) {
        std::cerr << "Error with ExposureModes enum for " << ip << ": " << e.what() << std::endl;
        return false;
    }
}

bool ParameterHandler::setShutterSpeed(const std::string& ip, int valueNum) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object shutterValueEnum = getPythonEnum("ShutterValues", valueNum);
        return callPythonMethod(controller, ip, "setShutterSpeed", shutterValueEnum);
    }
    catch (const std::exception& e) {
        std::cerr << "Error with ShutterValues enum for " << ip << ": " << e.what() << std::endl;
        return false;
    }
}

bool ParameterHandler::setManualACG(const std::string& ip, int valueNum) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object aegainsEnum = getPythonEnum("AEGains", valueNum);
        return callPythonMethod(controller, ip, "setManualACG", aegainsEnum);
    }
    catch (const std::exception& e) {
        std::cerr << "Error with AEGains enum for " << ip << ": " << e.what() << std::endl;
        return false;
    }
}

bool ParameterHandler::setWhiteBalanceMode(const std::string& ip, int modeValue) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object whiteBalanceModeEnum = getPythonEnum("WhiteBalanceModes", modeValue);
        return callPythonMethod(controller, ip, "setWhiteBalanceMode", whiteBalanceModeEnum);
    }
    catch (const std::exception& e) {
        std::cerr << "Error with WhiteBalanceModes enum for " << ip << ": " << e.what() << std::endl;
        return false;
    }
}

bool ParameterHandler::setIRMode(const std::string& ip, int modeValue) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object irModeEnum = getPythonEnum("IRModes", modeValue);
        return callPythonMethod(controller, ip, "setIRMode", irModeEnum);
    }
    catch (const std::exception& e) {
        std::cerr << "Error with IRModes enum for " << ip << ": " << e.what() << std::endl;
        return false;
    }
}

bool ParameterHandler::disableIR(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, ip, "disableIR");
}