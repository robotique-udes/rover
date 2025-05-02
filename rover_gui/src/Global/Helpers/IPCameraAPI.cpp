#include "Global/Helpers/IPCameraAPI.hpp"
#include <pybind11/embed.h>
#include <stdexcept>
#include <filesystem>
#include <unordered_map>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <fstream>

namespace py = pybind11;
namespace fs = std::filesystem;

// Helper class to wrap a Python object (with same visibility as pybind11 types)
class PYBIND11_EXPORT PyObjectWrapper {
public:
    py::object obj;

    PyObjectWrapper(const py::object& o) : obj(o) {}
    ~PyObjectWrapper() = default;
};

// Initialize the static Python interpreter
static bool pyInitialized = false;

// Set Python module path
void CameraController::setPythonModulePath(const std::string& path) {
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

// Helper function to check if a file exists
bool fileExists(const std::string& path) {
    std::ifstream file(path);
    return file.good();
}

// Constructor with ROS node integration
CameraController::CameraController(
    const std::string& node_name, 
    int port,
    const std::string& username, 
    const std::string& password) 
    : default_port(8999),    // FIXED: Use port 8999 to match test script
      default_username(username),
      default_password(password),
      logger(rclcpp::get_logger(node_name)) {

    // Initialize Python interpreter if not already initialized
    if (!pyInitialized) {
        py::initialize_interpreter();
        pyInitialized = true;
        
        // Add common paths to search for the Python module
        setPythonModulePath(".");  // Current directory
        setPythonModulePath(fs::current_path().string());  // Full current directory
        
        // Add ROS2 paths
        auto ros_workspace = std::getenv("COLCON_PREFIX_PATH");
        if (ros_workspace) {
            setPythonModulePath(ros_workspace);
        }
        
        // Add the build directory where we copied the Python module
        auto build_path = fs::current_path() / "build" / "rover_gui";
        if (fs::exists(build_path)) {
            setPythonModulePath(build_path.string());
        }
        
        // Try to find the Python file and add its directory to the path
        auto src_path = fs::current_path() / "src" / "Global" / "Helpers";
        if (fs::exists(src_path / "ipcamera_api.py")) {
            setPythonModulePath(src_path.string());
        }
        
        // Additional source paths to try
        auto root_src_path = fs::current_path() / "src";
        if (fs::exists(root_src_path)) {
            setPythonModulePath(root_src_path.string());
        }
        
        // Try direct parent directory
        auto parent_path = fs::current_path().parent_path();
        if (fs::exists(parent_path)) {
            setPythonModulePath(parent_path.string());
        }
        
        // Try to locate the file in various locations
        std::vector<std::string> potential_locations = {
            (fs::current_path() / "ipcamera_api.py").string(),
            (fs::current_path() / "build" / "rover_gui" / "ipcamera_api.py").string(),
            (fs::current_path() / "src" / "Global" / "Helpers" / "ipcamera_api.py").string(),
            (fs::current_path() / "src" / "ipcamera_api.py").string(),
            "/home/chris/ros2_ws/src/rover_gui/src/Global/Helpers/ipcamera_api.py",
            "/home/chris/ros2_ws/build/rover_gui/ipcamera_api.py",
            "/home/chris/ros2_ws/install/rover_gui/lib/rover_gui/ipcamera_api.py"
        };
        
        for (const auto& location : potential_locations) {
            if (fileExists(location)) {
                // Only log first found location to reduce logs
                RCLCPP_INFO(logger, "Found ipcamera_api.py at: %s", location.c_str());
                setPythonModulePath(fs::path(location).parent_path().string());
                
                // If the file exists but isn't in the build directory, try to copy it there
                auto build_file = fs::current_path() / "build" / "rover_gui" / "ipcamera_api.py";
                if (location != build_file.string() && fs::exists(fs::path(location))) {
                    try {
                        auto build_dir = fs::current_path() / "build" / "rover_gui";
                        if (!fs::exists(build_dir)) {
                            fs::create_directories(build_dir);
                        }
                        fs::copy_file(location, build_file, fs::copy_options::overwrite_existing);
                        setPythonModulePath(build_dir.string());
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(logger, "Failed to copy ipcamera_api.py to build directory: %s", e.what());
                    }
                }
                break; // Stop after finding the first valid location
            }
        }
    }
    
    // Try to load the module directly from the potential paths if import fails
    bool module_loaded = false;
    std::vector<std::string> potential_paths = {
        (fs::current_path() / "ipcamera_api.py").string(),
        (fs::current_path() / "build" / "rover_gui" / "ipcamera_api.py").string(),
        (fs::current_path() / "src" / "Global" / "Helpers" / "ipcamera_api.py").string(),
        (fs::current_path() / "src" / "ipcamera_api.py").string(),
        "/home/chris/ros2_ws/src/rover_gui/src/Global/Helpers/ipcamera_api.py",
        "/home/chris/ros2_ws/build/rover_gui/ipcamera_api.py",
        "/home/chris/ros2_ws/install/rover_gui/lib/rover_gui/ipcamera_api.py"
    };
    
    // Import the Python module 
    try {
        ipcamera_api_module = py::module::import("ipcamera_api");
        RCLCPP_INFO(logger, "Successfully imported ipcamera_api module");
        module_loaded = true;
    }
    catch (const py::error_already_set& e) {
        RCLCPP_ERROR(logger, "Failed to import ipcamera_api module: %s", e.what());
        
        // Try to load the module from file
        for (const auto& file_path : potential_paths) {
            if (fileExists(file_path)) {
                try {
                    // Use importlib to load from file path
                    py::module importlib = py::module::import("importlib.util");
                    py::object spec = importlib.attr("spec_from_file_location")("ipcamera_api", file_path);
                    if (!spec.is_none()) {
                        py::object module = importlib.attr("module_from_spec")(spec);
                        spec.attr("loader").attr("exec_module")(module);
                        ipcamera_api_module = module;
                        RCLCPP_INFO(logger, "Successfully loaded ipcamera_api module from file");
                        module_loaded = true;
                        break;
                    }
                } catch (const py::error_already_set& e2) {
                    RCLCPP_ERROR(logger, "Failed to load module from file: %s", e2.what());
                }
            }
        }
    }
    
    if (!module_loaded) {
        RCLCPP_ERROR(logger, "Failed to load ipcamera_api module. Camera functionality will be unavailable.");
    }
}

// Destructor
CameraController::~CameraController() {
    // Clear all camera connections
    std::lock_guard<std::mutex> lock(cacheMutex);
    connectionCache.clear();
}

// Helper to get Python enum instances
py::object CameraController::getPythonEnum(const std::string& enum_class, int value) {
    try {
        return ipcamera_api_module.attr(enum_class.c_str())(value);
    }
    catch (const py::error_already_set& e) {
        RCLCPP_ERROR(logger, "Python error getting %s enum value %d: %s", 
                    enum_class.c_str(), value, e.what());
        throw;
    }
}

// Get or create a controller for the given IP
PyObjectWrapper* CameraController::getOrCreateController(const std::string& ip) {
    // Check if the Python module was successfully loaded
    if (ipcamera_api_module.is_none()) {
        RCLCPP_ERROR(logger, "Python module was not loaded. Cannot create controller for %s", ip.c_str());
        return nullptr;
    }

    // Lock to prevent concurrent access to the cache
    std::lock_guard<std::mutex> lock(cacheMutex);
    
    // Check if we have a cached connection
    auto it = connectionCache.find(ip);
    if (it != connectionCache.end() && it->second.controller != nullptr) {
        return it->second.controller.get();
    }
    
    // Create a new connection
    try {
        // Create a Python dictionary for logging (ROS2 node logger)
        py::dict node_dict;
        node_dict["get_logger"] = py::cpp_function([this]() {
            // Create a logger dictionary with log methods
            py::dict logger_dict;
            logger_dict["info"] = py::cpp_function([this](const std::string& msg) {
                RCLCPP_INFO(this->logger, "%s", msg.c_str());
            });
            logger_dict["error"] = py::cpp_function([this](const std::string& msg) {
                RCLCPP_ERROR(this->logger, "%s", msg.c_str());
            });
            return logger_dict;
        });
        
        // Create a mock node object - direct assignment, no cast needed
        py::object node = node_dict;
        
        // Create an instance of the CameraController Python class with explicit port
        py::object controller = ipcamera_api_module.attr("CameraController")(
            node, ip, default_port, default_username, default_password, 5
        );
        
        // Store the Python controller instance in our cache
        CameraConnection conn;
        conn.controller = std::make_unique<PyObjectWrapper>(controller);
        
        connectionCache[ip] = std::move(conn);
        
        return connectionCache[ip].controller.get();
    }
    catch (const py::error_already_set& e) {
        RCLCPP_ERROR(logger, "Python error connecting to camera at %s: %s", ip.c_str(), e.what());
        return nullptr;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "C++ error connecting to camera at %s: %s", ip.c_str(), e.what());
        return nullptr;
    }
}

// Helper template function to call Python methods
template<typename... Args>
bool callPythonMethod(PyObjectWrapper* wrapper, const rclcpp::Logger& logger, 
                     const std::string& ip, const char* methodName, Args&&... args) {
    if (!wrapper) {
        RCLCPP_ERROR(logger, "Failed to get controller for camera at %s", ip.c_str());
        return false;
    }
    
    try {
        py::object result = wrapper->obj.attr(methodName)(std::forward<Args>(args)...);
        return py::cast<bool>(result);
    }
    catch (const py::error_already_set& e) {
        RCLCPP_ERROR(logger, "Python error calling %s on camera %s: %s", 
                    methodName, ip.c_str(), e.what());
        return false;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "C++ error calling %s on camera %s: %s", 
                    methodName, ip.c_str(), e.what());
        return false;
    }
}

// Implementation of camera parameter control methods
bool CameraController::setBrightness(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "setBrightness", value);
}

bool CameraController::setContrast(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "setContrast", value);
}

bool CameraController::setSaturation(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "setSaturation", value);
}

bool CameraController::setSharpness(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "setSharpness", value);
}

bool CameraController::setResolution(const std::string& ip, const std::string& value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "setResolution", value);
}

bool CameraController::setFrameRate(const std::string& ip, camera::FramerateValues value) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object frameRateEnum = getPythonEnum("FramerateValues", static_cast<int>(value));
        return callPythonMethod(controller, logger, ip, "setFrameRate", frameRateEnum);
    }
    catch (const py::error_already_set& e) {
        RCLCPP_ERROR(logger, "Python error with FramerateValues enum for %s: %s", 
                    ip.c_str(), e.what());
        return false;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error with FramerateValues enum for %s: %s", 
                    ip.c_str(), e.what());
        return false;
    }
}

bool CameraController::setBitrate(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "setBitrate", value);
}

bool CameraController::disableWideDynamicRange(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "disableWideDynamicRange");
}

bool CameraController::setWideDynamicRangeLevel(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "setWideDynamicRangeLevel", value);
}

bool CameraController::enableBackLight(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "enableBackLight");
}

bool CameraController::disableBackLight(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "disableBackLight");
}

bool CameraController::HorizontalMirror(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "HorizontalMirror");
}

bool CameraController::VerticalMirror(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "VerticalMirror");
}

bool CameraController::resetHorizontalMirror(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "resetHorizontalMirror");
}

bool CameraController::resetVerticalMirror(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "resetVerticalMirror");
}

bool CameraController::enableAntiFalseColor(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "enableAntiFalseColor");
}

bool CameraController::disableAntiFalseColor(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "disableAntiFalseColor");
}

bool CameraController::enableDigitalImageStabilizer(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "enableDigitalImageStabilizer");
}

bool CameraController::disableDigitalImageStabilizer(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "disableDigitalImageStabilizer");
}

bool CameraController::enableLensShadeCorrection(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "enableLensShadeCorrection");
}

bool CameraController::disableLensShadeCorrection(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "disableLensShadeCorrection");
}

bool CameraController::setLensDistortionCorrection(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "setLensDistortionCorrection", value);
}

bool CameraController::disableLensDistortionCorrection(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "disableLensDistortionCorrection");
}

bool CameraController::setAntiFog(const std::string& ip, int value) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "setAntiFog", value);
}

bool CameraController::disableAntiFog(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "disableAntiFog");
}

bool CameraController::setScene(const std::string& ip, camera::Scenes mode) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object scenesEnum = getPythonEnum("Scenes", static_cast<int>(mode));
        return callPythonMethod(controller, logger, ip, "setScene", scenesEnum);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error with Scenes enum for %s: %s", ip.c_str(), e.what());
        return false;
    }
}

bool CameraController::setExposureMode(const std::string& ip, camera::ExposureModes mode) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object exposureModeEnum = getPythonEnum("ExposureModes", static_cast<int>(mode));
        return callPythonMethod(controller, logger, ip, "setExposureMode", exposureModeEnum);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error with ExposureModes enum for %s: %s", ip.c_str(), e.what());
        return false;
    }
}

bool CameraController::setShutterSpeed(const std::string& ip, camera::ShutterValues value) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object shutterValueEnum = getPythonEnum("ShutterValues", static_cast<int>(value));
        return callPythonMethod(controller, logger, ip, "setShutterSpeed", shutterValueEnum);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error with ShutterValues enum for %s: %s", ip.c_str(), e.what());
        return false;
    }
}

bool CameraController::setManualACG(const std::string& ip, camera::AEGains value) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object aegainsEnum = getPythonEnum("AEGains", static_cast<int>(value));
        return callPythonMethod(controller, logger, ip, "setManualACG", aegainsEnum);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error with AEGains enum for %s: %s", ip.c_str(), e.what());
        return false;
    }
}

bool CameraController::setWhiteBalanceMode(const std::string& ip, camera::WhiteBalanceModes mode) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object whiteBalanceModeEnum = getPythonEnum("WhiteBalanceModes", static_cast<int>(mode));
        return callPythonMethod(controller, logger, ip, "setWhiteBalanceMode", whiteBalanceModeEnum);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error with WhiteBalanceModes enum for %s: %s", ip.c_str(), e.what());
        return false;
    }
}

bool CameraController::setIRMode(const std::string& ip, camera::IRModes mode) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        py::object irModeEnum = getPythonEnum("IRModes", static_cast<int>(mode));
        return callPythonMethod(controller, logger, ip, "setIRMode", irModeEnum);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error with IRModes enum for %s: %s", ip.c_str(), e.what());
        return false;
    }
}

bool CameraController::disableIR(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    return callPythonMethod(controller, logger, ip, "disableIR");
}

// Method to reset the parameter tracking for a camera
bool CameraController::resetParameters(const std::string& ip) {
    auto controller = getOrCreateController(ip);
    if (!controller) return false;
    
    try {
        controller->obj.attr("resetParameters")();
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger, "Error resetting parameters for %s: %s", ip.c_str(), e.what());
        return false;
    }
}