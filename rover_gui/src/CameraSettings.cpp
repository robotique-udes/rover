#include "CameraSettings.hpp"
#include "UI_CameraSettings.h"
#include "Global/Helpers/IPCameraAPI.hpp"
#include <QMessageBox>
#include <QRegularExpression>
#include <memory>

// Initialize the static camera controller
std::unique_ptr<ParameterHandler> CameraSettings::_cameraController = nullptr;

// Helper function to map resolution string to API value
QString mapResolutionToApi(const QString& resolution) {
    if (resolution == "2048x1536") return "1536p";
    if (resolution == "1920x1080") return "1080p";
    if (resolution == "1280x720") return "720p";
    if (resolution == "1280x960") return "960p";
    if (resolution == "720x576") return "576p";
    if (resolution == "640x480") return "480p";
    if (resolution == "720x480") return "w480p";
    return "720p"; 
}

// Map framerate string to enum value
int mapFramerateToApiValue(const QString& framerate) {
    if (framerate == "30 fps") return static_cast<int>(camera::FramerateValues::_30);
    if (framerate == "25 fps") return static_cast<int>(camera::FramerateValues::_25);
    if (framerate == "20 fps") return static_cast<int>(camera::FramerateValues::_20);
    if (framerate == "15 fps") return static_cast<int>(camera::FramerateValues::_15);
    if (framerate == "10 fps") return static_cast<int>(camera::FramerateValues::_10);
    if (framerate == "5 fps") return static_cast<int>(camera::FramerateValues::_5);
    if (framerate == "3 fps") return static_cast<int>(camera::FramerateValues::_3);
    return static_cast<int>(camera::FramerateValues::_30); // default
}

CameraSettings::CameraSettings(QWidget* parent) : 
    QDialog(parent),
    _ui(new Ui::CameraSettingsWidget)
{
    _ui->setupUi(this);
    setupUI();
    connectSignals();
    setupCameraSettingsConnections();
    initCameraController();

    setAttribute(Qt::WA_DeleteOnClose, false);
}

CameraSettings::~CameraSettings()
{
    delete _ui;
}

void CameraSettings::setupUI()
{
    setWindowTitle(tr("Camera Settings"));
    setModal(true);
    
    connect(_ui->cameraIpSelector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this](int) {
                _currentIp = _ui->cameraIpSelector->currentText();
                if (!_currentIp.isEmpty()) {
                    loadCameraSettings(_currentIp);
                }
            });
}

void CameraSettings::showSettings()
{
    
    QString currentIp = _ui->cameraIpSelector->currentText();
    if (!currentIp.isEmpty()) {
        _currentIp = currentIp;
        loadCameraSettings(_currentIp);
    }
    
    this->exec();
}

void CameraSettings::loadPredefinedIPs(const std::vector<QString>& ips)
{
    _ui->cameraIpSelector->clear();
    for (const auto& ip : ips) {
        if (!ip.isEmpty()) {
            _ui->cameraIpSelector->addItem(ip);
        }
    }
}

QString CameraSettings::extractIpFromUrl(const QString& url)
{
    static QRegularExpression ipRegex("rtsp://(?:[^:@]+(?::[^@]+)?@)?([^:/]+)");
    QRegularExpressionMatch match = ipRegex.match(url);
    if (match.hasMatch()) {
        return match.captured(1);
    }
    return QString();
}

void CameraSettings::connectSignals()
{
    // Connect the apply changes button
    connect(_ui->applyChangesBtn, &QPushButton::clicked, this, &CameraSettings::applyCameraSettings);
    
    // Connect the reset defaults button
    connect(_ui->resetDefaultsBtn, &QPushButton::clicked, this, &CameraSettings::resetCameraDefaults);
    
    // Connect the set defaults button
    connect(_ui->setDefaultsBtn, &QPushButton::clicked, this, &CameraSettings::setCustomDefaults);
}

void CameraSettings::saveCustomDefaults(const QString& ip)
{
    // Create a new defaults object or get existing one
    CameraDefaults defaults;
    
    // Save basic settings
    defaults.brightness = _ui->brightnessSlider->value();
    defaults.contrast = _ui->contrastSlider->value();
    defaults.saturation = _ui->saturationSlider->value();
    defaults.sharpness = _ui->sharpnessSlider->value();
    defaults.bitrate = _ui->bitrateSlider->value();
    defaults.resolutionIndex = _ui->resolutionComboBox->currentIndex();
    defaults.framerateIndex = _ui->framerateComboBox->currentIndex();
    
    // Save enhancement settings
    defaults.wdrEnabled = _ui->wdrCheckBox->isChecked();
    defaults.wdrLevel = _ui->wdrLevelSlider->value();
    defaults.backlightEnabled = _ui->backlightCheckBox->isChecked();
    defaults.antiFogEnabled = _ui->antiFogCheckBox->isChecked();
    defaults.antiFogLevel = _ui->antiFogLevelSlider->value();
    defaults.imageStabilizerEnabled = _ui->imageStabilizerCheckBox->isChecked();
    
    // Save correction settings
    defaults.horizontalMirrorEnabled = _ui->horizontalMirrorCheckBox->isChecked();
    defaults.verticalMirrorEnabled = _ui->verticalMirrorCheckBox->isChecked();
    defaults.antiFalseColorEnabled = _ui->antiFalseColorCheckBox->isChecked();
    defaults.lensShadeCorrectionEnabled = _ui->lensShadeCorrectionCheckBox->isChecked();
    defaults.lensDistortionCorrectionEnabled = _ui->lensDistortionCorrectionCheckBox->isChecked();
    defaults.lensDistortionCorrectionLevel = _ui->lensDistortionCorrectionSlider->value();
    
    // Save exposure settings
    defaults.sceneIndex = _ui->sceneComboBox->currentIndex();
    defaults.exposureModeIndex = _ui->exposureModeComboBox->currentIndex();
    defaults.shutterSpeedIndex = _ui->shutterSpeedComboBox->currentIndex();
    defaults.manualGainIndex = _ui->manualGainComboBox->currentIndex();
    
    // Save white balance and IR settings
    defaults.whiteBalanceModeIndex = _ui->whiteBalanceModeComboBox->currentIndex();
    defaults.irModeIndex = _ui->irModeComboBox->currentIndex();
    
    // Save checkbox states
    defaults.brightnessChecked = _ui->brightnessCheckbox->isChecked();
    defaults.contrastChecked = _ui->contrastCheckbox->isChecked();
    defaults.saturationChecked = _ui->saturationCheckbox->isChecked();
    defaults.sharpnessChecked = _ui->sharpnessCheckbox->isChecked();
    defaults.bitrateChecked = _ui->bitrateCheckbox->isChecked();
    defaults.resolutionChecked = _ui->resolutionCheckbox->isChecked();
    defaults.framerateChecked = _ui->framerateCheckbox->isChecked();
    defaults.sceneChecked = _ui->sceneCheckbox->isChecked();
    defaults.exposureModeChecked = _ui->exposureModeCheckbox->isChecked();
    defaults.shutterSpeedChecked = _ui->shutterSpeedCheckbox->isChecked();
    defaults.manualGainChecked = _ui->manualGainCheckbox->isChecked();
    defaults.whiteBalanceModeChecked = _ui->whiteBalanceModeCheckbox->isChecked();
    defaults.irModeChecked = _ui->irModeCheckbox->isChecked();
    
    // Store the custom defaults for this IP
    _customDefaults[ip] = defaults;
    
    LOG_INFO("CameraSettings", "Custom defaults saved for " + ip);
}

void CameraSettings::loadCustomDefaults(const QString& ip)
{
    // Check if custom defaults exist for this IP
    auto it = _customDefaults.find(ip);
    if (it == _customDefaults.end()) {
        // No custom defaults, load factory defaults
        loadCameraSettings(ip);
        return;
    }
    
    // Get the custom defaults
    const CameraDefaults& defaults = it->second;
    
    LOG_INFO("CameraSettings", "Loading custom defaults for " + ip);
    
    // Apply basic settings
    _ui->brightnessSlider->setValue(defaults.brightness);
    _ui->contrastSlider->setValue(defaults.contrast);
    _ui->saturationSlider->setValue(defaults.saturation);
    _ui->sharpnessSlider->setValue(defaults.sharpness);
    _ui->bitrateSlider->setValue(defaults.bitrate);
    _ui->resolutionComboBox->setCurrentIndex(defaults.resolutionIndex);
    _ui->framerateComboBox->setCurrentIndex(defaults.framerateIndex);
    
    // Apply checkbox states
    _ui->brightnessCheckbox->setChecked(defaults.brightnessChecked);
    _ui->contrastCheckbox->setChecked(defaults.contrastChecked);
    _ui->saturationCheckbox->setChecked(defaults.saturationChecked);
    _ui->sharpnessCheckbox->setChecked(defaults.sharpnessChecked);
    _ui->bitrateCheckbox->setChecked(defaults.bitrateChecked);
    _ui->resolutionCheckbox->setChecked(defaults.resolutionChecked);
    _ui->framerateCheckbox->setChecked(defaults.framerateChecked);
    
    // Apply enhancement settings
    _ui->wdrCheckBox->setChecked(defaults.wdrEnabled);
    _ui->wdrLevelSlider->setValue(defaults.wdrLevel);
    _ui->wdrLevelSlider->setEnabled(defaults.wdrEnabled);
    _ui->wdrLevelSpinBox->setEnabled(defaults.wdrEnabled);
    
    _ui->backlightCheckBox->setChecked(defaults.backlightEnabled);
    
    _ui->antiFogCheckBox->setChecked(defaults.antiFogEnabled);
    _ui->antiFogLevelSlider->setValue(defaults.antiFogLevel);
    _ui->antiFogLevelSlider->setEnabled(defaults.antiFogEnabled);
    _ui->antiFogLevelSpinBox->setEnabled(defaults.antiFogEnabled);
    
    _ui->imageStabilizerCheckBox->setChecked(defaults.imageStabilizerEnabled);
    
    // Apply correction settings
    _ui->horizontalMirrorCheckBox->setChecked(defaults.horizontalMirrorEnabled);
    _ui->verticalMirrorCheckBox->setChecked(defaults.verticalMirrorEnabled);
    _ui->antiFalseColorCheckBox->setChecked(defaults.antiFalseColorEnabled);
    _ui->lensShadeCorrectionCheckBox->setChecked(defaults.lensShadeCorrectionEnabled);
    
    _ui->lensDistortionCorrectionCheckBox->setChecked(defaults.lensDistortionCorrectionEnabled);
    _ui->lensDistortionCorrectionSlider->setValue(defaults.lensDistortionCorrectionLevel);
    _ui->lensDistortionCorrectionSlider->setEnabled(defaults.lensDistortionCorrectionEnabled);
    _ui->lensDistortionCorrectionSpinBox->setEnabled(defaults.lensDistortionCorrectionEnabled);
    
    // Apply exposure settings
    _ui->sceneCheckbox->setChecked(defaults.sceneChecked);
    _ui->sceneComboBox->setCurrentIndex(defaults.sceneIndex);
    
    _ui->exposureModeCheckbox->setChecked(defaults.exposureModeChecked);
    _ui->exposureModeComboBox->setCurrentIndex(defaults.exposureModeIndex);
    
    _ui->shutterSpeedCheckbox->setChecked(defaults.shutterSpeedChecked);
    _ui->shutterSpeedComboBox->setCurrentIndex(defaults.shutterSpeedIndex);
    
    _ui->manualGainCheckbox->setChecked(defaults.manualGainChecked);
    _ui->manualGainComboBox->setCurrentIndex(defaults.manualGainIndex);
    
    // Apply white balance and IR settings
    _ui->whiteBalanceModeCheckbox->setChecked(defaults.whiteBalanceModeChecked);
    _ui->whiteBalanceModeComboBox->setCurrentIndex(defaults.whiteBalanceModeIndex);
    
    _ui->irModeCheckbox->setChecked(defaults.irModeChecked);
    _ui->irModeComboBox->setCurrentIndex(defaults.irModeIndex);
    
    // Update dependent controls based on exposure mode
    onExposureModeChanged(_ui->exposureModeComboBox->currentIndex());
}

void CameraSettings::setCustomDefaults()
{
    QString ip = _ui->cameraIpSelector->currentText();
    if (ip.isEmpty()) {
        QMessageBox::warning(this, "Invalid IP", "Please enter a valid IP address");
        return;
    }
    
    // Confirm with user
    auto response = QMessageBox::question(this, "Set Custom Defaults", 
                                       "Save current settings as defaults for this camera?",
                                       QMessageBox::Yes | QMessageBox::No);
    
    if (response == QMessageBox::Yes) {
        saveCustomDefaults(ip);
        QMessageBox::information(this, "Defaults Saved", 
                             "Current settings have been saved as defaults for this camera.");
    }
}

void CameraSettings::initCameraController()
{
    if (!_cameraController) {
        try {
            // Create the camera controller with default parameters
            _cameraController = std::make_unique<ParameterHandler>(
                "admin",  // default username
                "admin",  // default password
                8999      // default port
            );
            LOG_INFO("CameraSettings", "Camera controller initialized");
        }
        catch (const std::exception& e) {
            LOG_ERROR("CameraSettings", "Failed to initialize camera controller: " + QString(e.what()));
        }
    }
}

void CameraSettings::setupCameraSettingsConnections()
{
    // Connect sliders and spinboxes bidirectionally
    connectSliderAndSpinBox("brightness");
    connectSliderAndSpinBox("contrast");
    connectSliderAndSpinBox("saturation");
    connectSliderAndSpinBox("sharpness");
    connectSliderAndSpinBox("bitrate");
    connectSliderAndSpinBox("wdrLevel");
    connectSliderAndSpinBox("antiFogLevel");
    connectSliderAndSpinBox("lensDistortionCorrection");
    
    // Connect exposure mode combobox to enable/disable related controls
    connect(_ui->exposureModeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &CameraSettings::onExposureModeChanged);
            
    // Handle WDR checkbox to enable/disable WDR level controls
    connect(_ui->wdrCheckBox, &QCheckBox::toggled, [this](bool checked) {
        _ui->wdrLevelSlider->setEnabled(checked);
        _ui->wdrLevelSpinBox->setEnabled(checked);
    });
    
    // Handle Anti-Fog checkbox to enable/disable level controls
    connect(_ui->antiFogCheckBox, &QCheckBox::toggled, [this](bool checked) {
        _ui->antiFogLevelSlider->setEnabled(checked);
        _ui->antiFogLevelSpinBox->setEnabled(checked);
    });
    
    // Handle Lens Distortion Correction checkbox
    connect(_ui->lensDistortionCorrectionCheckBox, &QCheckBox::toggled, [this](bool checked) {
        _ui->lensDistortionCorrectionSlider->setEnabled(checked);
        _ui->lensDistortionCorrectionSpinBox->setEnabled(checked);
    });
}

void CameraSettings::onExposureModeChanged(int index)
{
    // Enable/disable shutter speed and manual gain based on exposure mode
    bool shutterEnabled = (index == 2); // Shutter mode
    bool manualGainEnabled = (index == 1); // Manual mode
    
    _ui->shutterSpeedCheckbox->setEnabled(shutterEnabled);
    _ui->shutterSpeedComboBox->setEnabled(shutterEnabled);
    
    _ui->manualGainCheckbox->setEnabled(manualGainEnabled);
    _ui->manualGainComboBox->setEnabled(manualGainEnabled);
}

void CameraSettings::connectSliderAndSpinBox(const QString& baseName)
{
    auto slider = findChild<QSlider*>(baseName + "Slider");
    auto spinBox = findChild<QSpinBox*>(baseName + "SpinBox");
    
    if (slider && spinBox) {
        // Connect slider to spinbox
        connect(slider, &QSlider::valueChanged, spinBox, &QSpinBox::setValue);
        
        // Connect spinbox to slider
        connect(spinBox, QOverload<int>::of(&QSpinBox::valueChanged), slider, &QSlider::setValue);
    }
}

void CameraSettings::loadCameraSettings(const QString& ip)
{
    LOG_INFO("CameraSettings", "Initializing camera settings for " + ip);
    
    // Check if custom defaults exist for this IP
    auto it = _customDefaults.find(ip);
    if (it != _customDefaults.end()) {
        // Use custom defaults
        loadCustomDefaults(ip);
        return;
    }
    
    // Initialize all controls with factory default values
    
    // Basic settings
    _ui->brightnessSlider->setValue(50);
    _ui->contrastSlider->setValue(50);
    _ui->saturationSlider->setValue(50);
    _ui->sharpnessSlider->setValue(50);
    _ui->bitrateSlider->setValue(4096);
    _ui->resolutionComboBox->setCurrentIndex(0);
    _ui->framerateComboBox->setCurrentIndex(0);
    
    // Uncheck all checkboxes by default
    _ui->brightnessCheckbox->setChecked(false);
    _ui->contrastCheckbox->setChecked(false);
    _ui->saturationCheckbox->setChecked(false);
    _ui->sharpnessCheckbox->setChecked(false);
    _ui->bitrateCheckbox->setChecked(false);
    _ui->resolutionCheckbox->setChecked(false);
    _ui->framerateCheckbox->setChecked(false);
    
    _ui->wdrCheckBox->setChecked(false);
    _ui->backlightCheckBox->setChecked(false);
    _ui->antiFogCheckBox->setChecked(false);
    _ui->imageStabilizerCheckBox->setChecked(false);
    
    _ui->horizontalMirrorCheckBox->setChecked(false);
    _ui->verticalMirrorCheckBox->setChecked(false);
    _ui->antiFalseColorCheckBox->setChecked(false);
    _ui->lensShadeCorrectionCheckBox->setChecked(false);
    _ui->lensDistortionCorrectionCheckBox->setChecked(false);
    
    _ui->sceneCheckbox->setChecked(false);
    _ui->exposureModeCheckbox->setChecked(false);
    _ui->shutterSpeedCheckbox->setChecked(false);
    _ui->manualGainCheckbox->setChecked(false);
    
    _ui->whiteBalanceModeCheckbox->setChecked(false);
    _ui->irModeCheckbox->setChecked(false);
    
    // Initialize dependent control states
    _ui->wdrLevelSlider->setEnabled(false);
    _ui->wdrLevelSpinBox->setEnabled(false);
    _ui->wdrLevelSlider->setValue(50);
    
    _ui->antiFogLevelSlider->setEnabled(false);
    _ui->antiFogLevelSpinBox->setEnabled(false);
    _ui->antiFogLevelSlider->setValue(50);
    
    _ui->lensDistortionCorrectionSlider->setEnabled(false);
    _ui->lensDistortionCorrectionSpinBox->setEnabled(false);
    _ui->lensDistortionCorrectionSlider->setValue(50);
    
    // Set default values for comboboxes
    _ui->sceneComboBox->setCurrentIndex(0);
    _ui->exposureModeComboBox->setCurrentIndex(0);
    _ui->shutterSpeedComboBox->setCurrentIndex(8); // 1/50
    _ui->manualGainComboBox->setCurrentIndex(0); // 1x
    _ui->whiteBalanceModeComboBox->setCurrentIndex(0); // Auto
    _ui->irModeComboBox->setCurrentIndex(0); // Auto
    
    // Update dependent controls based on exposure mode
    onExposureModeChanged(_ui->exposureModeComboBox->currentIndex());
}

void CameraSettings::resetCameraDefaults()
{
    // Get current IP
    QString ip = _ui->cameraIpSelector->currentText();
    if (ip.isEmpty()) {
        QMessageBox::warning(this, "Invalid IP", "Please enter a valid IP address");
        return;
    }
    
    // Confirm with user
    auto response = QMessageBox::question(this, "Reset Defaults", 
                                       "Are you sure you want to reset all camera settings to defaults?",
                                       QMessageBox::Yes | QMessageBox::No);
    
    if (response == QMessageBox::Yes) {
        // Load custom defaults if they exist, otherwise load factory defaults
        loadCustomDefaults(ip);
    }
}

void CameraSettings::applyCameraSettings()
{
    if (!_cameraController) {
        LOG_ERROR("CameraSettings", "Camera controller not initialized");
        QMessageBox::warning(this, "Error", "Camera controller not initialized");
        return;
    }
    
    QString ip = _ui->cameraIpSelector->currentText();
    if (ip.isEmpty()) {
        QMessageBox::warning(this, "Invalid IP", "Please enter a valid IP address");
        return;
    }
    
    // First, check if the camera is reachable using the API's built-in check
    if (!_cameraController->isCameraReachable(ip.toStdString(), true)) {
        QMessageBox::critical(this, "Connection Error", 
                           "Cannot connect to camera at " + ip + ". Please check that the camera is online and try again.");
        return;
    }
    
    QApplication::setOverrideCursor(Qt::WaitCursor);
    
    try {
        // Apply basic settings
        applyCameraBasicSettings(ip);
        
        // Apply image enhancement settings
        applyCameraImageEnhancementSettings(ip);
        
        // Apply image correction settings
        applyCameraImageCorrectionSettings(ip);
        
        // Apply exposure settings
        applyCameraExposureSettings(ip);
        
        // Apply white balance and IR settings
        applyCameraWhiteBalanceAndIRSettings(ip);
        
        QApplication::restoreOverrideCursor();
        LOG_INFO("CameraSettings", "Successfully applied camera settings for " + ip);
        QMessageBox::information(this, "Settings Applied", 
                              "Successfully applied camera settings.");
    }
    catch (const std::exception& e) {
        QApplication::restoreOverrideCursor();
        LOG_ERROR("CameraSettings", "Exception applying camera settings: " + QString(e.what()));
        QMessageBox::critical(this, "Settings Error", 
                           "Error applying camera settings: " + QString(e.what()));
    }
    catch (...) {
        QApplication::restoreOverrideCursor();
        LOG_ERROR("CameraSettings", "Unknown exception applying camera settings");
        QMessageBox::critical(this, "Settings Error", 
                           "Unknown error applying camera settings. The camera may be offline or unreachable.");
    }
}

void CameraSettings::applyCameraBasicSettings(const QString& ip)
{
    // Brightness
    if (_ui->brightnessCheckbox->isChecked()) {
        int brightnessValue = _ui->brightnessSlider->value() * 255 / 100; // Scale 0-100 to 0-255
        _cameraController->setBrightness(ip.toStdString(), brightnessValue);
        LOG_INFO("CameraSettings", "Applied brightness: " + QString::number(brightnessValue));
    }
    
    // Contrast
    if (_ui->contrastCheckbox->isChecked()) {
        int contrastValue = _ui->contrastSlider->value() * 255 / 100; // Scale 0-100 to 0-255
        _cameraController->setContrast(ip.toStdString(), contrastValue);
        LOG_INFO("CameraSettings", "Applied contrast: " + QString::number(contrastValue));
    }
    
    // Saturation
    if (_ui->saturationCheckbox->isChecked()) {
        int saturationValue = _ui->saturationSlider->value() * 255 / 100; // Scale 0-100 to 0-255
        _cameraController->setSaturation(ip.toStdString(), saturationValue);
        LOG_INFO("CameraSettings", "Applied saturation: " + QString::number(saturationValue));
    }
    
    // Sharpness
    if (_ui->sharpnessCheckbox->isChecked()) {
        int sharpnessValue = _ui->sharpnessSlider->value() * 255 / 100; // Scale 0-100 to 0-255
        _cameraController->setSharpness(ip.toStdString(), sharpnessValue);
        LOG_INFO("CameraSettings", "Applied sharpness: " + QString::number(sharpnessValue));
    }
    
    // Bitrate
    if (_ui->bitrateCheckbox->isChecked()) {
        int bitrateValue = _ui->bitrateSlider->value();
        _cameraController->setBitrate(ip.toStdString(), bitrateValue);
        LOG_INFO("CameraSettings", "Applied bitrate: " + QString::number(bitrateValue));
    }
    
    // Resolution
    if (_ui->resolutionCheckbox->isChecked()) {
        QString resolutionText = _ui->resolutionComboBox->currentText();
        QString apiResolution = mapResolutionToApi(resolutionText);
        _cameraController->setResolution(ip.toStdString(), apiResolution.toStdString());
        LOG_INFO("CameraSettings", "Applied resolution: " + apiResolution);
    }
    
    // Framerate
    if (_ui->framerateCheckbox->isChecked()) {
        QString framerateText = _ui->framerateComboBox->currentText();
        int frameRateValue = mapFramerateToApiValue(framerateText);
        _cameraController->setFrameRate(ip.toStdString(), frameRateValue);
        LOG_INFO("CameraSettings", "Applied framerate: " + framerateText);
    }
}

void CameraSettings::applyCameraImageEnhancementSettings(const QString& ip)
{
    // Wide Dynamic Range
    if (_ui->wdrCheckBox->isChecked()) {
        int wdrLevel = _ui->wdrLevelSlider->value() * 255 / 100; // Scale 0-100 to 0-255
        _cameraController->setWideDynamicRangeLevel(ip.toStdString(), wdrLevel);
        LOG_INFO("CameraSettings", "Applied WDR level: " + QString::number(wdrLevel));
    } else if (_ui->wdrCheckBox->isEnabled()) {
        _cameraController->disableWideDynamicRange(ip.toStdString());
        LOG_INFO("CameraSettings", "Disabled WDR");
    }
    
    // Backlight Compensation
    if (_ui->backlightCheckBox->isChecked()) {
        _cameraController->enableBackLight(ip.toStdString());
        LOG_INFO("CameraSettings", "Enabled backlight compensation");
    } else if (_ui->backlightCheckBox->isEnabled()) {
        _cameraController->disableBackLight(ip.toStdString());
        LOG_INFO("CameraSettings", "Disabled backlight compensation");
    }
    
    // Anti-Fog
    if (_ui->antiFogCheckBox->isChecked()) {
        int antiFogLevel = _ui->antiFogLevelSlider->value() * 255 / 100; // Scale 0-100 to 0-255
        _cameraController->setAntiFog(ip.toStdString(), antiFogLevel);
        LOG_INFO("CameraSettings", "Applied anti-fog level: " + QString::number(antiFogLevel));
    } else if (_ui->antiFogCheckBox->isEnabled()) {
        _cameraController->disableAntiFog(ip.toStdString());
        LOG_INFO("CameraSettings", "Disabled anti-fog");
    }
    
    // Digital Image Stabilizer
    if (_ui->imageStabilizerCheckBox->isChecked()) {
        _cameraController->enableDigitalImageStabilizer(ip.toStdString());
        LOG_INFO("CameraSettings", "Enabled digital image stabilizer");
    } else if (_ui->imageStabilizerCheckBox->isEnabled()) {
        _cameraController->disableDigitalImageStabilizer(ip.toStdString());
        LOG_INFO("CameraSettings", "Disabled digital image stabilizer");
    }
}

void CameraSettings::applyCameraImageCorrectionSettings(const QString& ip)
{
    // Horizontal Mirror
    if (_ui->horizontalMirrorCheckBox->isChecked()) {
        _cameraController->HorizontalMirror(ip.toStdString());
        LOG_INFO("CameraSettings", "Enabled horizontal mirror");
    } else if (_ui->horizontalMirrorCheckBox->isEnabled()) {
        _cameraController->resetHorizontalMirror(ip.toStdString());
        LOG_INFO("CameraSettings", "Disabled horizontal mirror");
    }
    
    // Vertical Mirror
    if (_ui->verticalMirrorCheckBox->isChecked()) {
        _cameraController->VerticalMirror(ip.toStdString());
        LOG_INFO("CameraSettings", "Enabled vertical mirror");
    } else if (_ui->verticalMirrorCheckBox->isEnabled()) {
        _cameraController->resetVerticalMirror(ip.toStdString());
        LOG_INFO("CameraSettings", "Disabled vertical mirror");
    }
    
    // Anti-False Color
    if (_ui->antiFalseColorCheckBox->isChecked()) {
        _cameraController->enableAntiFalseColor(ip.toStdString());
        LOG_INFO("CameraSettings", "Enabled anti-false color");
    } else if (_ui->antiFalseColorCheckBox->isEnabled()) {
        _cameraController->disableAntiFalseColor(ip.toStdString());
        LOG_INFO("CameraSettings", "Disabled anti-false color");
    }
    
    // Lens Shade Correction
    if (_ui->lensShadeCorrectionCheckBox->isChecked()) {
        _cameraController->enableLensShadeCorrection(ip.toStdString());
        LOG_INFO("CameraSettings", "Enabled lens shade correction");
    } else if (_ui->lensShadeCorrectionCheckBox->isEnabled()) {
        _cameraController->disableLensShadeCorrection(ip.toStdString());
        LOG_INFO("CameraSettings", "Disabled lens shade correction");
    }
    
    // Lens Distortion Correction
    if (_ui->lensDistortionCorrectionCheckBox->isChecked()) {
        int ldcLevel = _ui->lensDistortionCorrectionSlider->value() * 255 / 100; // Scale 0-100 to 0-255
        _cameraController->setLensDistortionCorrection(ip.toStdString(), ldcLevel);
        LOG_INFO("CameraSettings", "Applied lens distortion correction level: " + QString::number(ldcLevel));
    } else if (_ui->lensDistortionCorrectionCheckBox->isEnabled()) {
        _cameraController->disableLensDistortionCorrection(ip.toStdString());
        LOG_INFO("CameraSettings", "Disabled lens distortion correction");
    }
}

void CameraSettings::applyCameraExposureSettings(const QString& ip)
{
    // Scene
    if (_ui->sceneCheckbox->isChecked()) {
        int sceneValue = (_ui->sceneComboBox->currentIndex() == 0) ? 
                         static_cast<int>(camera::Scenes::INDOOR) : 
                         static_cast<int>(camera::Scenes::OUTDOOR);
        _cameraController->setScene(ip.toStdString(), sceneValue);
        LOG_INFO("CameraSettings", "Applied scene: " + _ui->sceneComboBox->currentText());
    }
    
    // Exposure Mode
    if (_ui->exposureModeCheckbox->isChecked()) {
        int modeValue;
        switch (_ui->exposureModeComboBox->currentIndex()) {
            case 0: modeValue = static_cast<int>(camera::ExposureModes::SCENE); break;
            case 1: modeValue = static_cast<int>(camera::ExposureModes::MANUAL); break;
            case 2: modeValue = static_cast<int>(camera::ExposureModes::SHUTTER); break;
            default: modeValue = static_cast<int>(camera::ExposureModes::SCENE); break;
        }
        _cameraController->setExposureMode(ip.toStdString(), modeValue);
        LOG_INFO("CameraSettings", "Applied exposure mode: " + _ui->exposureModeComboBox->currentText());
    }
    
    // Shutter Speed (only if exposure mode is SHUTTER)
    if (_ui->shutterSpeedCheckbox->isChecked() && _ui->shutterSpeedCheckbox->isEnabled()) {
        // Map index to ShutterValues enum
        static const std::vector<int> shutterValues = {
            static_cast<int>(camera::ShutterValues::_1_8000),
            static_cast<int>(camera::ShutterValues::_1_6000),
            static_cast<int>(camera::ShutterValues::_1_4000),
            static_cast<int>(camera::ShutterValues::_1_2000),
            static_cast<int>(camera::ShutterValues::_1_1000),
            static_cast<int>(camera::ShutterValues::_1_500),
            static_cast<int>(camera::ShutterValues::_1_250),
            static_cast<int>(camera::ShutterValues::_1_100),
            static_cast<int>(camera::ShutterValues::_1_50),
            static_cast<int>(camera::ShutterValues::_1_25),
            static_cast<int>(camera::ShutterValues::_1_10),
            static_cast<int>(camera::ShutterValues::_1_5),
            static_cast<int>(camera::ShutterValues::_1)
        };
        
        int index = _ui->shutterSpeedComboBox->currentIndex();
        if (index >= 0 && index < static_cast<int>(shutterValues.size())) {
            _cameraController->setShutterSpeed(ip.toStdString(), shutterValues[index]);
            LOG_INFO("CameraSettings", "Applied shutter speed: " + _ui->shutterSpeedComboBox->currentText());
        }
    }
    
    // Manual ACG (only if exposure mode is MANUAL)
    if (_ui->manualGainCheckbox->isChecked() && _ui->manualGainCheckbox->isEnabled()) {
        // Map index to AEGains enum
        static const std::vector<int> gainValues = {
            static_cast<int>(camera::AEGains::_1X),
            static_cast<int>(camera::AEGains::_2X),
            static_cast<int>(camera::AEGains::_4X),
            static_cast<int>(camera::AEGains::_8X),
            static_cast<int>(camera::AEGains::_16X),
            static_cast<int>(camera::AEGains::_32X),
            static_cast<int>(camera::AEGains::_64X)
        };
        
        int index = _ui->manualGainComboBox->currentIndex();
        if (index >= 0 && index < static_cast<int>(gainValues.size())) {
            _cameraController->setManualACG(ip.toStdString(), gainValues[index]);
            LOG_INFO("CameraSettings", "Applied manual gain: " + _ui->manualGainComboBox->currentText());
        }
    }
}

void CameraSettings::applyCameraWhiteBalanceAndIRSettings(const QString& ip)
{
    // White Balance Mode
    if (_ui->whiteBalanceModeCheckbox->isChecked()) {
        static const std::vector<int> wbModes = {
            static_cast<int>(camera::WhiteBalanceModes::AUTO),
            static_cast<int>(camera::WhiteBalanceModes::MANUAL),
            static_cast<int>(camera::WhiteBalanceModes::INDOOR),
            static_cast<int>(camera::WhiteBalanceModes::OUTDOOR),
            static_cast<int>(camera::WhiteBalanceModes::SUNLIGHT)
        };
        
        int index = _ui->whiteBalanceModeComboBox->currentIndex();
        if (index >= 0 && index < static_cast<int>(wbModes.size())) {
            _cameraController->setWhiteBalanceMode(ip.toStdString(), wbModes[index]);
            LOG_INFO("CameraSettings", "Applied white balance mode: " + _ui->whiteBalanceModeComboBox->currentText());
        }
    }
    
    // IR Mode
    if (_ui->irModeCheckbox->isChecked()) {
        if (_ui->irModeComboBox->currentIndex() == 3) {
            // "Disabled" option
            _cameraController->disableIR(ip.toStdString());
            LOG_INFO("CameraSettings", "Disabled IR");
        } else {
            static const std::vector<int> irModes = {
                static_cast<int>(camera::IRModes::AUTO),
                static_cast<int>(camera::IRModes::DAY),
                static_cast<int>(camera::IRModes::NIGHT)
            };
            
            int index = _ui->irModeComboBox->currentIndex();
            if (index >= 0 && index < static_cast<int>(irModes.size())) {
                _cameraController->setIRMode(ip.toStdString(), irModes[index]);
                LOG_INFO("CameraSettings", "Applied IR mode: " + _ui->irModeComboBox->currentText());
            }
        }
    }
}