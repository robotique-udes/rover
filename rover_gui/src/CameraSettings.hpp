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
    void showSettings(const QString& streamUrl = QString());

private slots:
    void applyCameraSettings();
    void resetCameraDefaults();
    void onExposureModeChanged(int index);

private:
    void setupUI();
    void connectSignals();
    void setupCameraSettingsConnections();
    void connectSliderAndSpinBox(const QString& baseName);
    void loadCameraSettings(const QString& ip);
    void applyCameraBasicSettings(const QString& ip);
    void applyCameraImageEnhancementSettings(const QString& ip);
    void applyCameraImageCorrectionSettings(const QString& ip);
    void applyCameraExposureSettings(const QString& ip);
    void applyCameraWhiteBalanceAndIRSettings(const QString& ip);
    static void initCameraController();
    QString extractIpFromUrl(const QString& url);

    Ui::CameraSettingsWidget* _ui;
    QString _currentIp;
    QString _streamUrl;
    
    // Camera controller - static singleton instance
    static std::unique_ptr<ParameterHandler> _cameraController;
};

#endif // CAMERA_SETTINGS_HPP