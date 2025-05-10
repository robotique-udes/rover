#ifndef CAMERA_SETTINGS_HPP
#define CAMERA_SETTINGS_HPP

#include <QDialog>
#include <QWidget>
#include <QString>
#include <memory>
#include <QComboBox>
#include <QCheckBox>
#include <QSlider>
#include <QSpinBox>
#include <QPushButton>
#include <QLineEdit>
#include <vector>
#include <map>

// Forward declaration for ParameterHandler
namespace camera {
    enum class FramerateValues;
    enum class ShutterValues;
    enum class AEGains;
    enum class Scenes;
    enum class ExposureModes;
    enum class WhiteBalanceModes;
    enum class IRModes;
}
class ParameterHandler;

namespace Ui {
    class CameraSettingsWidget;
}

class CameraSettings : public QDialog
{
    Q_OBJECT

public:
    explicit CameraSettings(QWidget* parent = nullptr);
    ~CameraSettings();

    void loadPredefinedIPs(const std::vector<QString>& ips);
    void showSettings();

private slots:
    void applyCameraSettings();
    void resetCameraDefaults();
    void setCustomDefaults();
    void onExposureModeChanged(int index);

private:
    void setupUI();
    void connectSignals();
    void setupCameraSettingsConnections();
    void connectSliderAndSpinBox(const QString& baseName);
    void loadCameraSettings(const QString& ip);
    void loadCustomDefaults(const QString& ip);
    void saveCustomDefaults(const QString& ip);
    void applyCameraBasicSettings(const QString& ip);
    void applyCameraImageEnhancementSettings(const QString& ip);
    void applyCameraImageCorrectionSettings(const QString& ip);
    void applyCameraExposureSettings(const QString& ip);
    void applyCameraWhiteBalanceAndIRSettings(const QString& ip);
    static void initCameraController();
    QString extractIpFromUrl(const QString& url);

    // Structure to store custom defaults for a particular IP
    struct CameraDefaults {
        // Basic settings
        int brightness = 50;
        int contrast = 50;
        int saturation = 50;
        int sharpness = 50;
        int bitrate = 4096;
        int resolutionIndex = 0;
        int framerateIndex = 0;
        
        // Enhancement settings
        bool wdrEnabled = false;
        int wdrLevel = 50;
        bool backlightEnabled = false;
        bool antiFogEnabled = false;
        int antiFogLevel = 50;
        bool imageStabilizerEnabled = false;
        
        // Correction settings
        bool horizontalMirrorEnabled = false;
        bool verticalMirrorEnabled = false;
        bool antiFalseColorEnabled = false;
        bool lensShadeCorrectionEnabled = false;
        bool lensDistortionCorrectionEnabled = false;
        int lensDistortionCorrectionLevel = 50;
        
        // Exposure settings
        int sceneIndex = 0;
        int exposureModeIndex = 0;
        int shutterSpeedIndex = 8; // Default to 1/50
        int manualGainIndex = 0;   // Default to 1x
        
        // White balance and IR settings
        int whiteBalanceModeIndex = 0;
        int irModeIndex = 0;
        
        // Checkbox states
        bool brightnessChecked = false;
        bool contrastChecked = false;
        bool saturationChecked = false;
        bool sharpnessChecked = false;
        bool bitrateChecked = false;
        bool resolutionChecked = false;
        bool framerateChecked = false;
        bool sceneChecked = false;
        bool exposureModeChecked = false;
        bool shutterSpeedChecked = false;
        bool manualGainChecked = false;
        bool whiteBalanceModeChecked = false;
        bool irModeChecked = false;
    };
    
    Ui::CameraSettingsWidget* _ui;
    QString _currentIp;
    QString _streamUrl;
    
    // Map to store custom defaults for each camera IP
    std::map<QString, CameraDefaults> _customDefaults;
    
    // Camera controller - static singleton instance
    static std::unique_ptr<ParameterHandler> _cameraController;
};

#endif // CAMERA_SETTINGS_HPP