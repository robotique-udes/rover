#ifndef SECONDARY_WINDOW_HPP
#define SECONDARY_WINDOW_HPP

#include "QVideoPlayer/QVideoManagerWidget.hpp"
#include <QMainWindow>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <memory>
#include <vector>
#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "CameraSettings.hpp" // New include for the camera settings

namespace Ui {
    class SecondaryWindow; // Forward declaration of the UI class
}
#include <QShortcut>

class SecondaryWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit SecondaryWindow(std::shared_ptr<rclcpp::Node> node = nullptr);
    ~SecondaryWindow();

private slots:
    void onLayoutChange(int index_);
    void onStreamStateChanged(bool running_, int streamIndex_);
    void showCameraSettings(); 

private:
    // UI setup methods
    void setupUI(void);
    void closeEvent(QCloseEvent* event) override;
    void loadPredefinedStreams(void);
    void initializeStreams(void);
    void updateLayout(void);
    void initializeRosServicesForWidgets(void);
    QString extractIpFromUrl(const QString& url);
    
    // Stream container helpers
    QWidget* createStreamContainer(int streamIndex_);
    QShortcut _closeShortCut;
    void addStreamSelector(RtspPlayerWidget* widget_, int position_);
    
    // UI member
    Ui::SecondaryWindow* _ui;
    
    // ROS components
    std::shared_ptr<rclcpp::Node> _node;
    
    // Stream configuration
    struct PredefinedStream {
        QString name;
        QString url;
    };
    std::vector<PredefinedStream> _predefinedStreams;
    
    // Active stream management
    struct ActiveStream {
        std::unique_ptr<RtspPlayerWidget> widget;
        std::unique_ptr<QLabel> headerLabel;
        int predefinedStreamIndex = -1; // -1 = None
        bool isRunning = false;
    };
    std::vector<ActiveStream> _activeStreams;
    
    // Camera settings dialog
    CameraSettings* _cameraSettings;
    
    // Configuration
    static constexpr int MAX_STREAMS = 6;
    int _currentStreamIndex = 0;
};

#endif  // SECONDARY_WINDOW_HPP