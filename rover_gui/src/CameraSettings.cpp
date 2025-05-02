#include "CameraSettings.hpp"
#include "UI_CameraSettings.h"
#include "QRtspPlayer/QLoggingMacros.hpp"
#include "Global/Helpers/IPCameraAPI.hpp"
#include <QMessageBox>
#include <QRegularExpression>
#include <memory>

// Initialize the static camera controller
std::unique_ptr<CameraController> CameraSettings::_cameraController = nullptr;

// Helper function to map resolution string to API value
QString mapResolutionToApi(const QString& resolution) {
    if (resolution == "1920x1080") return "1080p";
    if (resolution == "1280x720") return "720p";
    if (resolution == "800x600") return "960p"; // closest match
    if (resolution == "640x480") return "480p";
    return "720p"; // default
}

// Map framerate string to API enum value
camera::FramerateValues mapFramerateToApi(const QString& framerate) {
    if (framerate == "30 fps") return camera::FramerateValues::_30;
    if (framerate == "25 fps") return camera::FramerateValues::_25;
    if (framerate == "20 fps") return camera::FramerateValues::_20;
    if (framerate == "15 fps") return camera::FramerateValues::_15;
    if (framerate == "10 fps") return camera::FramerateValues::_10;
    if (framerate == "5 fps") return camera::FramerateValues::_5;
    if (framerate == "3 fps") return camera::FramerateValues::_3;
    return camera::FramerateValues::_30; // default
}

CameraSettings::CameraSettings(QWidget* parent) : 
    QDialog(parent),
    _ui(new Ui::CameraSettingsWidget),
    _isModified(false)
{
    _ui->setupUi(this);
    setupUI();
    connectSignals();
    setupCameraSettingsConnections();
    initCameraController();
}

CameraSettings::~CameraSettings()
{
    delete _ui;
}

void CameraSettings::setupUI()
{
    setWindowTitle(tr("Camera Settings"));
    setModal(true);
}

void CameraSettings::showSettings(const QString& streamUrl)
{
    _streamUrl = streamUrl;
    _currentIp = extractIpFromUrl(streamUrl);
    
    if (!_currentIp.isEmpty()) {
        // Check if IP is already in the list
        int index = _ui->cameraIpSelector->findText(_currentIp);
        if (index >= 0) {
            _ui->cameraIpSelector->setCurrentIndex(index);
        } else {
            _ui->cameraIpSelector->addItem(_currentIp);
            _ui->cameraIpSelector->setCurrentText(_currentIp);
        }
        
        // Load settings for this IP
        loadCameraSettings(_currentIp);
    }
    
    _isModified = false;
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
    // Connect back button
    connect(_ui->backFromSettingsBtn, &QPushButton::clicked, this, &CameraSettings::reject);
    
    // Connect the apply changes button
    connect(_ui->applyChangesBtn, &QPushButton::clicked, this, &CameraSettings::applyCameraSettings);
    
    // Connect the reset defaults button
    connect(_ui->resetDefaultsBtn, &QPushButton::clicked, this, &CameraSettings::resetCameraDefaults);
}

void CameraSettings::initCameraController()
{
    if (!_cameraController) {
        try {
            // Create the camera controller with the ROS node name
            _cameraController = std::make_unique<CameraController>(
                "camera_controller",
                80,  // default port
                "admin",  // default username
                "admin"   // default password
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
    
    // Connect checkboxes to enable/disable their related controls
    connectCheckBoxToControls("wdrCheckBox", {"wdrLevelSlider", "wdrLevelSpinBox"});
    connectCheckBoxToControls("antiFogCheckBox", {"antiFogLevelSlider", "antiFogLevelSpinBox"});
    connectCheckBoxToControls("lensDistortionCorrectionCheckBox", 
                              {"lensDistortionCorrectionSlider", "lensDistortionCorrectionSpinBox"});
    
    // Connect exposure mode combobox to enable/disable related controls
    connect(_ui->exposureModeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &CameraSettings::onExposureModeChanged);
            
    // Connect all controls to track modifications
    // For sliders
    QList<QSlider*> sliders = findChildren<QSlider*>();
    for (auto slider : sliders) {
        connect(slider, &QSlider::valueChanged, this, &CameraSettings::onSettingChanged);
    }
    
    // For checkboxes
    QList<QCheckBox*> checkboxes = findChildren<QCheckBox*>();
    for (auto checkbox : checkboxes) {
        connect(checkbox, &QCheckBox::toggled, this, &CameraSettings::onSettingChanged);
    }
    
    // For comboboxes
    QList<QComboBox*> comboboxes = findChildren<QComboBox*>();
    for (auto combobox : comboboxes) {
        connect(combobox, QOverload<int>::of(&QComboBox::currentIndexChanged), 
                this, &CameraSettings::onSettingChanged);
    }
}

void CameraSettings::onSettingChanged()
{
    QObject* sender = QObject::sender();
    if (!sender) return;
    
    QString controlName = sender->objectName();
    QString paramName;
    
    // Map control name to parameter name
    if (controlName.endsWith("Slider") || controlName.endsWith("SpinBox")) {
        paramName = controlName.left(controlName.lastIndexOf("Slider") != -1 ? 
                                    controlName.lastIndexOf("Slider") : 
                                    controlName.lastIndexOf("SpinBox"));
    }
    else if (controlName.endsWith("CheckBox")) {
        paramName = controlName.left(controlName.lastIndexOf("CheckBox"));
    }
    else if (controlName.endsWith("ComboBox")) {
        paramName = controlName.left(controlName.lastIndexOf("ComboBox"));
    }
    
    if (!paramName.isEmpty() && _parameterStates.find(paramName) != _parameterStates.end()) {
        QVariant newValue;
        
        // Get value based on control type
        if (auto slider = qobject_cast<QSlider*>(sender))
            newValue = slider->value();
        else if (auto spinBox = qobject_cast<QSpinBox*>(sender))
            newValue = spinBox->value();
        else if (auto checkBox = qobject_cast<QCheckBox*>(sender))
            newValue = checkBox->isChecked();
        else if (auto comboBox = qobject_cast<QComboBox*>(sender))
            newValue = comboBox->currentIndex();
        
        // Update parameter state
        _parameterStates[paramName].modified = true;
        _parameterStates[paramName].value = newValue;
        
        LOG_INFO("CameraSettings", "Parameter changed: " + paramName);
    }
    
    _isModified = true;
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

void CameraSettings::connectCheckBoxToControls(const QString& checkBoxName, const QStringList& controlNames)
{
    auto checkBox = findChild<QCheckBox*>(checkBoxName);
    if (!checkBox) return;
    
    connect(checkBox, &QCheckBox::toggled, this, [this, controlNames](bool checked) {
        for (const auto& controlName : controlNames) {
            auto control = findChild<QWidget*>(controlName);
            if (control) {
                control->setEnabled(checked);
            }
        }
    });
}

void CameraSettings::loadCameraSettings(const QString& ip)
{
    LOG_INFO("CameraSettings", "Initializing camera settings for " + ip);
    
    // Clear previous parameter states
    _parameterStates.clear();
    
    // Initialize all parameters as unmodified with sentinel values
    _parameterStates["brightness"] = {false, QVariant(-1)};
    _parameterStates["contrast"] = {false, QVariant(-1)};
    _parameterStates["saturation"] = {false, QVariant(-1)};
    _parameterStates["sharpness"] = {false, QVariant(-1)};
    _parameterStates["bitrate"] = {false, QVariant(-1)};
    _parameterStates["wdr"] = {false, QVariant(-1)};
    _parameterStates["wdrLevel"] = {false, QVariant(-1)};
    _parameterStates["backlight"] = {false, QVariant(-1)};
    _parameterStates["horizontalMirror"] = {false, QVariant(-1)};
    _parameterStates["verticalMirror"] = {false, QVariant(-1)};
    _parameterStates["antiFalseColor"] = {false, QVariant(-1)};
    _parameterStates["lensShadeCorrection"] = {false, QVariant(-1)};
    _parameterStates["lensDistortionCorrection"] = {false, QVariant(-1)};
    _parameterStates["lensDistortionCorrectionLevel"] = {false, QVariant(-1)};
    _parameterStates["antiFog"] = {false, QVariant(-1)};
    _parameterStates["antiFogLevel"] = {false, QVariant(-1)};
    _parameterStates["imageStabilizer"] = {false, QVariant(-1)};
    _parameterStates["resolution"] = {false, QVariant("")};
    _parameterStates["framerate"] = {false, QVariant(-1)};
    _parameterStates["scene"] = {false, QVariant(-1)};
    _parameterStates["exposureMode"] = {false, QVariant(-1)};
    _parameterStates["shutterSpeed"] = {false, QVariant(-1)};
    _parameterStates["manualGain"] = {false, QVariant(-1)};
    _parameterStates["whiteBalanceMode"] = {false, QVariant(-1)};
    _parameterStates["irMode"] = {false, QVariant(-1)};
    
    // Set UI controls to default displayed values, but don't mark as modified
    // (these are just visual defaults, not sent to camera)
    auto brightnessSlider = findChild<QSlider*>("brightnessSlider");
    if (brightnessSlider) brightnessSlider->setValue(50);
    
    auto contrastSlider = findChild<QSlider*>("contrastSlider");
    if (contrastSlider) contrastSlider->setValue(50);
    
    auto saturationSlider = findChild<QSlider*>("saturationSlider");
    if (saturationSlider) saturationSlider->setValue(50);
    
    auto sharpnessSlider = findChild<QSlider*>("sharpnessSlider");
    if (sharpnessSlider) sharpnessSlider->setValue(50);
    
    auto bitrateSlider = findChild<QSlider*>("bitrateSlider");
    if (bitrateSlider) bitrateSlider->setValue(4096);
    
    // Set default values for checkboxes
    auto wdrCheckBox = findChild<QCheckBox*>("wdrCheckBox");
    if (wdrCheckBox) wdrCheckBox->setChecked(false);
    
    auto backlightCheckBox = findChild<QCheckBox*>("backlightCheckBox");
    if (backlightCheckBox) backlightCheckBox->setChecked(false);
    
    auto antiFogCheckBox = findChild<QCheckBox*>("antiFogCheckBox");
    if (antiFogCheckBox) antiFogCheckBox->setChecked(false);
    
    auto imageStabilizerCheckBox = findChild<QCheckBox*>("imageStabilizerCheckBox");
    if (imageStabilizerCheckBox) imageStabilizerCheckBox->setChecked(false);
    
    auto horizontalMirrorCheckBox = findChild<QCheckBox*>("horizontalMirrorCheckBox");
    if (horizontalMirrorCheckBox) horizontalMirrorCheckBox->setChecked(false);
    
    auto verticalMirrorCheckBox = findChild<QCheckBox*>("verticalMirrorCheckBox");
    if (verticalMirrorCheckBox) verticalMirrorCheckBox->setChecked(false);
    
    auto antiFalseColorCheckBox = findChild<QCheckBox*>("antiFalseColorCheckBox");
    if (antiFalseColorCheckBox) antiFalseColorCheckBox->setChecked(false);
    
    auto lensShadeCorrectionCheckBox = findChild<QCheckBox*>("lensShadeCorrectionCheckBox");
    if (lensShadeCorrectionCheckBox) lensShadeCorrectionCheckBox->setChecked(false);
    
    auto lensDistortionCorrectionCheckBox = findChild<QCheckBox*>("lensDistortionCorrectionCheckBox");
    if (lensDistortionCorrectionCheckBox) lensDistortionCorrectionCheckBox->setChecked(false);
    
    // Set default values for comboboxes
    auto resolutionComboBox = findChild<QComboBox*>("resolutionComboBox");
    if (resolutionComboBox) resolutionComboBox->setCurrentIndex(0);  // 1920x1080
    
    auto framerateComboBox = findChild<QComboBox*>("framerateComboBox");
    if (framerateComboBox) framerateComboBox->setCurrentIndex(0);  // 30 fps
    
    auto sceneComboBox = findChild<QComboBox*>("sceneComboBox");
    if (sceneComboBox) sceneComboBox->setCurrentIndex(0);  // Indoor
    
    auto exposureModeComboBox = findChild<QComboBox*>("exposureModeComboBox");
    if (exposureModeComboBox) {
        exposureModeComboBox->setCurrentIndex(0);  // Scene
        onExposureModeChanged(0);  // Update dependent controls
    }
    
    auto whiteBalanceModeComboBox = findChild<QComboBox*>("whiteBalanceModeComboBox");
    if (whiteBalanceModeComboBox) whiteBalanceModeComboBox->setCurrentIndex(0);  // Auto
    
    auto irModeComboBox = findChild<QComboBox*>("irModeComboBox");
    if (irModeComboBox) irModeComboBox->setCurrentIndex(0);  // Auto
    
    // Reset the modified flag since we just loaded values
    _isModified = false;
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
        // Clear parameter states
        for (auto& param : _parameterStates) {
            param.second.modified = false;
        }
        
        loadCameraSettings(ip);
    }
}

void CameraSettings::applyCameraSettings()
{
    if (!_cameraController) {
        LOG_ERROR("CameraSettings", "Camera controller not initialized");
        return;
    }
    
    QString ip = _ui->cameraIpSelector->currentText();
    if (ip.isEmpty()) {
        QMessageBox::warning(this, "Invalid IP", "Please enter a valid IP address");
        return;
    }
    
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
        
        LOG_INFO("CameraSettings", "Successfully applied modified camera settings for " + ip);
        QMessageBox::information(this, "Settings Applied", 
                              "Successfully applied camera settings.");
                               
        // Reset the modified flags
        for (auto& param : _parameterStates) {
            param.second.modified = false;
        }
        
        // Reset the overall modified flag
        _isModified = false;
    }
    catch (const std::exception& e) {
        LOG_ERROR("CameraSettings", "Exception applying camera settings: " + QString(e.what()));
        QMessageBox::critical(this, "Settings Error", 
                           "Error applying camera settings: " + QString(e.what()));
    }
}

void CameraSettings::applyCameraBasicSettings(const QString& ip)
{
    // Brightness
    if (_parameterStates["brightness"].modified) {
        auto brightnessSlider = findChild<QSlider*>("brightnessSlider");
        if (brightnessSlider) {
            int brightnessValue = brightnessSlider->value() * 255 / 100; // Scale 0-100 to 0-255
            _cameraController->setBrightness(ip.toStdString(), brightnessValue);
            LOG_INFO("CameraSettings", "Applied brightness: " + QString::number(brightnessValue));
        }
    }
    
    // Contrast
    if (_parameterStates["contrast"].modified) {
        auto contrastSlider = findChild<QSlider*>("contrastSlider");
        if (contrastSlider) {
            int contrastValue = contrastSlider->value() * 255 / 100; // Scale 0-100 to 0-255
            _cameraController->setContrast(ip.toStdString(), contrastValue);
            LOG_INFO("CameraSettings", "Applied contrast: " + QString::number(contrastValue));
        }
    }
    
    // Saturation
    if (_parameterStates["saturation"].modified) {
        auto saturationSlider = findChild<QSlider*>("saturationSlider");
        if (saturationSlider) {
            int saturationValue = saturationSlider->value() * 255 / 100; // Scale 0-100 to 0-255
            _cameraController->setSaturation(ip.toStdString(), saturationValue);
            LOG_INFO("CameraSettings", "Applied saturation: " + QString::number(saturationValue));
        }
    }
    
    // Sharpness
    if (_parameterStates["sharpness"].modified) {
        auto sharpnessSlider = findChild<QSlider*>("sharpnessSlider");
        if (sharpnessSlider) {
            int sharpnessValue = sharpnessSlider->value() * 255 / 100; // Scale 0-100 to 0-255
            _cameraController->setSharpness(ip.toStdString(), sharpnessValue);
            LOG_INFO("CameraSettings", "Applied sharpness: " + QString::number(sharpnessValue));
        }
    }
    
    // Bitrate
    if (_parameterStates["bitrate"].modified) {
        auto bitrateSlider = findChild<QSlider*>("bitrateSlider");
        if (bitrateSlider) {
            int bitrateValue = bitrateSlider->value();
            _cameraController->setBitrate(ip.toStdString(), bitrateValue);
            LOG_INFO("CameraSettings", "Applied bitrate: " + QString::number(bitrateValue));
        }
    }
    
    // Resolution
    if (_parameterStates["resolution"].modified) {
        auto resolutionComboBox = findChild<QComboBox*>("resolutionComboBox");
        if (resolutionComboBox) {
            QString resolutionText = resolutionComboBox->currentText();
            QString apiResolution = mapResolutionToApi(resolutionText);
            _cameraController->setResolution(ip.toStdString(), apiResolution.toStdString());
            LOG_INFO("CameraSettings", "Applied resolution: " + apiResolution);
        }
    }
    
    // Framerate
    if (_parameterStates["framerate"].modified) {
        auto framerateComboBox = findChild<QComboBox*>("framerateComboBox");
        if (framerateComboBox) {
            QString framerateText = framerateComboBox->currentText();
            camera::FramerateValues framerate = mapFramerateToApi(framerateText);
            _cameraController->setFrameRate(ip.toStdString(), framerate);
            LOG_INFO("CameraSettings", "Applied framerate: " + framerateText);
        }
    }
}

void CameraSettings::applyCameraImageEnhancementSettings(const QString& ip)
{
    // Wide Dynamic Range
    if (_parameterStates["wdr"].modified) {
        auto wdrCheckBox = findChild<QCheckBox*>("wdrCheckBox");
        auto wdrLevelSlider = findChild<QSlider*>("wdrLevelSlider");
        
        if (wdrCheckBox) {
            if (wdrCheckBox->isChecked() && wdrLevelSlider) {
                int wdrLevel = wdrLevelSlider->value() * 255 / 100; // Scale 0-100 to 0-255
                _cameraController->setWideDynamicRangeLevel(ip.toStdString(), wdrLevel);
                LOG_INFO("CameraSettings", "Applied WDR level: " + QString::number(wdrLevel));
            } else {
                _cameraController->disableWideDynamicRange(ip.toStdString());
                LOG_INFO("CameraSettings", "Disabled WDR");
            }
        }
    } else if (_parameterStates["wdrLevel"].modified) {
        auto wdrCheckBox = findChild<QCheckBox*>("wdrCheckBox");
        auto wdrLevelSlider = findChild<QSlider*>("wdrLevelSlider");
        
        if (wdrCheckBox && wdrCheckBox->isChecked() && wdrLevelSlider) {
            int wdrLevel = wdrLevelSlider->value() * 255 / 100; // Scale 0-100 to 0-255
            _cameraController->setWideDynamicRangeLevel(ip.toStdString(), wdrLevel);
            LOG_INFO("CameraSettings", "Applied WDR level: " + QString::number(wdrLevel));
        }
    }
    
    // Backlight Compensation
    if (_parameterStates["backlight"].modified) {
        auto backlightCheckBox = findChild<QCheckBox*>("backlightCheckBox");
        if (backlightCheckBox) {
            if (backlightCheckBox->isChecked()) {
                _cameraController->enableBackLight(ip.toStdString());
                LOG_INFO("CameraSettings", "Enabled backlight compensation");
            } else {
                _cameraController->disableBackLight(ip.toStdString());
                LOG_INFO("CameraSettings", "Disabled backlight compensation");
            }
        }
    }
    
    // Anti-Fog
    if (_parameterStates["antiFog"].modified) {
        auto antiFogCheckBox = findChild<QCheckBox*>("antiFogCheckBox");
        auto antiFogLevelSlider = findChild<QSlider*>("antiFogLevelSlider");
        
        if (antiFogCheckBox) {
            if (antiFogCheckBox->isChecked() && antiFogLevelSlider) {
                int antiFogLevel = antiFogLevelSlider->value() * 255 / 100; // Scale 0-100 to 0-255
                _cameraController->setAntiFog(ip.toStdString(), antiFogLevel);
                LOG_INFO("CameraSettings", "Applied anti-fog level: " + QString::number(antiFogLevel));
            } else {
                _cameraController->disableAntiFog(ip.toStdString());
                LOG_INFO("CameraSettings", "Disabled anti-fog");
            }
        }
    } else if (_parameterStates["antiFogLevel"].modified) {
        auto antiFogCheckBox = findChild<QCheckBox*>("antiFogCheckBox");
        auto antiFogLevelSlider = findChild<QSlider*>("antiFogLevelSlider");
        
        if (antiFogCheckBox && antiFogCheckBox->isChecked() && antiFogLevelSlider) {
            int antiFogLevel = antiFogLevelSlider->value() * 255 / 100; // Scale 0-100 to 0-255
            _cameraController->setAntiFog(ip.toStdString(), antiFogLevel);
            LOG_INFO("CameraSettings", "Applied anti-fog level: " + QString::number(antiFogLevel));
        }
    }
    
    // Digital Image Stabilizer
    if (_parameterStates["imageStabilizer"].modified) {
        auto imageStabilizerCheckBox = findChild<QCheckBox*>("imageStabilizerCheckBox");
        if (imageStabilizerCheckBox) {
            if (imageStabilizerCheckBox->isChecked()) {
                _cameraController->enableDigitalImageStabilizer(ip.toStdString());
                LOG_INFO("CameraSettings", "Enabled digital image stabilizer");
            } else {
                _cameraController->disableDigitalImageStabilizer(ip.toStdString());
                LOG_INFO("CameraSettings", "Disabled digital image stabilizer");
            }
        }
    }
}

void CameraSettings::applyCameraImageCorrectionSettings(const QString& ip)
{
    // Horizontal Mirror
    if (_parameterStates["horizontalMirror"].modified) {
        auto horizontalMirrorCheckBox = findChild<QCheckBox*>("horizontalMirrorCheckBox");
        if (horizontalMirrorCheckBox) {
            if (horizontalMirrorCheckBox->isChecked()) {
                _cameraController->HorizontalMirror(ip.toStdString());
                LOG_INFO("CameraSettings", "Enabled horizontal mirror");
            } else {
                _cameraController->resetHorizontalMirror(ip.toStdString());
                LOG_INFO("CameraSettings", "Disabled horizontal mirror");
            }
        }
    }
    
    // Vertical Mirror
    if (_parameterStates["verticalMirror"].modified) {
        auto verticalMirrorCheckBox = findChild<QCheckBox*>("verticalMirrorCheckBox");
        if (verticalMirrorCheckBox) {
            if (verticalMirrorCheckBox->isChecked()) {
                _cameraController->VerticalMirror(ip.toStdString());
                LOG_INFO("CameraSettings", "Enabled vertical mirror");
            } else {
                _cameraController->resetVerticalMirror(ip.toStdString());
                LOG_INFO("CameraSettings", "Disabled vertical mirror");
            }
        }
    }
    
    // Anti-False Color
    if (_parameterStates["antiFalseColor"].modified) {
        auto antiFalseColorCheckBox = findChild<QCheckBox*>("antiFalseColorCheckBox");
        if (antiFalseColorCheckBox) {
            if (antiFalseColorCheckBox->isChecked()) {
                _cameraController->enableAntiFalseColor(ip.toStdString());
                LOG_INFO("CameraSettings", "Enabled anti-false color");
            } else {
                _cameraController->disableAntiFalseColor(ip.toStdString());
                LOG_INFO("CameraSettings", "Disabled anti-false color");
            }
        }
    }
    
    // Lens Shade Correction
    if (_parameterStates["lensShadeCorrection"].modified) {
        auto lensShadeCorrectionCheckBox = findChild<QCheckBox*>("lensShadeCorrectionCheckBox");
        if (lensShadeCorrectionCheckBox) {
            if (lensShadeCorrectionCheckBox->isChecked()) {
                _cameraController->enableLensShadeCorrection(ip.toStdString());
                LOG_INFO("CameraSettings", "Enabled lens shade correction");
            } else {
                _cameraController->disableLensShadeCorrection(ip.toStdString());
                LOG_INFO("CameraSettings", "Disabled lens shade correction");
            }
        }
    }
    
    // Lens Distortion Correction
    if (_parameterStates["lensDistortionCorrection"].modified) {
        auto lensDistortionCorrectionCheckBox = findChild<QCheckBox*>("lensDistortionCorrectionCheckBox");
        auto lensDistortionCorrectionSlider = findChild<QSlider*>("lensDistortionCorrectionSlider");
        
        if (lensDistortionCorrectionCheckBox) {
            if (lensDistortionCorrectionCheckBox->isChecked() && lensDistortionCorrectionSlider) {
                int ldcLevel = lensDistortionCorrectionSlider->value() * 255 / 100; // Scale 0-100 to 0-255
                _cameraController->setLensDistortionCorrection(ip.toStdString(), ldcLevel);
                LOG_INFO("CameraSettings", "Applied lens distortion correction level: " + QString::number(ldcLevel));
            } else {
                _cameraController->disableLensDistortionCorrection(ip.toStdString());
                LOG_INFO("CameraSettings", "Disabled lens distortion correction");
            }
        }
    } else if (_parameterStates["lensDistortionCorrectionLevel"].modified) {
        auto lensDistortionCorrectionCheckBox = findChild<QCheckBox*>("lensDistortionCorrectionCheckBox");
        auto lensDistortionCorrectionSlider = findChild<QSlider*>("lensDistortionCorrectionSlider");
        
        if (lensDistortionCorrectionCheckBox && lensDistortionCorrectionCheckBox->isChecked() && lensDistortionCorrectionSlider) {
            int ldcLevel = lensDistortionCorrectionSlider->value() * 255 / 100; // Scale 0-100 to 0-255
            _cameraController->setLensDistortionCorrection(ip.toStdString(), ldcLevel);
            LOG_INFO("CameraSettings", "Applied lens distortion correction level: " + QString::number(ldcLevel));
        }
    }
}

void CameraSettings::applyCameraExposureSettings(const QString& ip)
{
    // Scene
    if (_parameterStates["scene"].modified) {
        auto sceneComboBox = findChild<QComboBox*>("sceneComboBox");
        if (sceneComboBox) {
            camera::Scenes scene = (sceneComboBox->currentIndex() == 0) ? 
                                 camera::Scenes::INDOOR : camera::Scenes::OUTDOOR;
            _cameraController->setScene(ip.toStdString(), scene);
            LOG_INFO("CameraSettings", "Applied scene: " + sceneComboBox->currentText());
        }
    }
    
    // Exposure Mode
    if (_parameterStates["exposureMode"].modified) {
        auto exposureModeComboBox = findChild<QComboBox*>("exposureModeComboBox");
        if (exposureModeComboBox) {
            camera::ExposureModes mode;
            switch (exposureModeComboBox->currentIndex()) {
                case 0: mode = camera::ExposureModes::SCENE; break;
                case 1: mode = camera::ExposureModes::MANUAL; break;
                case 2: mode = camera::ExposureModes::SHUTTER; break;
                default: mode = camera::ExposureModes::SCENE; break;
            }
            _cameraController->setExposureMode(ip.toStdString(), mode);
            LOG_INFO("CameraSettings", "Applied exposure mode: " + exposureModeComboBox->currentText());
        }
    }
    
    // Shutter Speed (only if exposure mode is SHUTTER)
    if (_parameterStates["shutterSpeed"].modified) {
        auto exposureModeComboBox = findChild<QComboBox*>("exposureModeComboBox");
        auto shutterSpeedComboBox = findChild<QComboBox*>("shutterSpeedComboBox");
        if (exposureModeComboBox && exposureModeComboBox->currentIndex() == 2 && shutterSpeedComboBox) {
            // Map index to ShutterValues enum
            static const std::vector<camera::ShutterValues> shutterValues = {
                camera::ShutterValues::_1_8000,
                camera::ShutterValues::_1_6000,
                camera::ShutterValues::_1_4000,
                camera::ShutterValues::_1_2000,
                camera::ShutterValues::_1_1000,
                camera::ShutterValues::_1_500,
                camera::ShutterValues::_1_250,
                camera::ShutterValues::_1_100,
                camera::ShutterValues::_1_50,
                camera::ShutterValues::_1_25,
                camera::ShutterValues::_1_10,
                camera::ShutterValues::_1_5,
                camera::ShutterValues::_1
            };
            
            int index = shutterSpeedComboBox->currentIndex();
            if (index >= 0 && index < static_cast<int>(shutterValues.size())) {
                _cameraController->setShutterSpeed(ip.toStdString(), shutterValues[index]);
                LOG_INFO("CameraSettings", "Applied shutter speed: " + shutterSpeedComboBox->currentText());
            }
        }
    }
    
    // Manual ACG (only if exposure mode is MANUAL)
    if (_parameterStates["manualGain"].modified) {
        auto exposureModeComboBox = findChild<QComboBox*>("exposureModeComboBox");
        auto manualGainComboBox = findChild<QComboBox*>("manualGainComboBox");
        if (exposureModeComboBox && exposureModeComboBox->currentIndex() == 1 && manualGainComboBox) {
            // Map index to AEGains enum
            static const std::vector<camera::AEGains> gainValues = {
                camera::AEGains::_1X,
                camera::AEGains::_2X,
                camera::AEGains::_4X,
                camera::AEGains::_8X,
                camera::AEGains::_16X,
                camera::AEGains::_32X,
                camera::AEGains::_64X
            };
            
            int index = manualGainComboBox->currentIndex();
            if (index >= 0 && index < static_cast<int>(gainValues.size())) {
                _cameraController->setManualACG(ip.toStdString(), gainValues[index]);
                LOG_INFO("CameraSettings", "Applied manual gain: " + manualGainComboBox->currentText());
            }
        }
    }
}

void CameraSettings::applyCameraWhiteBalanceAndIRSettings(const QString& ip)
{
    // White Balance Mode
    if (_parameterStates["whiteBalanceMode"].modified) {
        auto whiteBalanceModeComboBox = findChild<QComboBox*>("whiteBalanceModeComboBox");
        if (whiteBalanceModeComboBox) {
            static const std::vector<camera::WhiteBalanceModes> wbModes = {
                camera::WhiteBalanceModes::AUTO,
                camera::WhiteBalanceModes::MANUAL,
                camera::WhiteBalanceModes::INDOOR,
                camera::WhiteBalanceModes::OUTDOOR,
                camera::WhiteBalanceModes::SUNLIGHT
            };
            
            int index = whiteBalanceModeComboBox->currentIndex();
            if (index >= 0 && index < static_cast<int>(wbModes.size())) {
                _cameraController->setWhiteBalanceMode(ip.toStdString(), wbModes[index]);
                LOG_INFO("CameraSettings", "Applied white balance mode: " + whiteBalanceModeComboBox->currentText());
            }
        }
    }
    
    // IR Mode
    if (_parameterStates["irMode"].modified) {
        auto irModeComboBox = findChild<QComboBox*>("irModeComboBox");
        if (irModeComboBox) {
            if (irModeComboBox->currentIndex() == 3) {
                // "Disabled" option
                _cameraController->disableIR(ip.toStdString());
                LOG_INFO("CameraSettings", "Disabled IR");
            } else {
                static const std::vector<camera::IRModes> irModes = {
                    camera::IRModes::AUTO,
                    camera::IRModes::DAY,
                    camera::IRModes::NIGHT
                };
                
                int index = irModeComboBox->currentIndex();
                if (index >= 0 && index < static_cast<int>(irModes.size())) {
                    _cameraController->setIRMode(ip.toStdString(), irModes[index]);
                    LOG_INFO("CameraSettings", "Applied IR mode: " + irModeComboBox->currentText());
                }
            }
        }
    }
}

void CameraSettings::onExposureModeChanged(int index)
{
    auto shutterSpeedComboBox = findChild<QComboBox*>("shutterSpeedComboBox");
    auto manualGainComboBox = findChild<QComboBox*>("manualGainComboBox");
    
    if (shutterSpeedComboBox && manualGainComboBox) {
        // Enable/disable controls based on exposure mode
        switch (index) {
            case 0: // Scene
                shutterSpeedComboBox->setEnabled(false);
                manualGainComboBox->setEnabled(false);
                break;
                
            case 1: // Manual
                shutterSpeedComboBox->setEnabled(false);
                manualGainComboBox->setEnabled(true);
                break;
                
            case 2: // Shutter
                shutterSpeedComboBox->setEnabled(true);
                manualGainComboBox->setEnabled(false);
                break;
                
            default:
                shutterSpeedComboBox->setEnabled(false);
                manualGainComboBox->setEnabled(false);
                break;
        }
    }
}