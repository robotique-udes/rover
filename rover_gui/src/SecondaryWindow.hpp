#ifndef SECONDARY_WINDOW_HPP
#define SECONDARY_WINDOW_HPP

#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"
#include "CameraSettings.hpp" 
#include "QVideoPlayer/QVideoManagerWidget.hpp"

#include <QMainWindow>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <memory>
#include <QShortcut>
#include <vector>

namespace Ui {
    class SecondaryWindow; 
}

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
    void setupUI(void);
    void closeEvent(QCloseEvent* event) override;
    void loadPredefinedStreams(void);
    void initializeStreams(void);
    void updateLayout(void);
    void initializeRosServicesForWidgets(void);
    QString extractIpFromUrl(const QString& url);
    
    QWidget* createStreamContainer(int streamIndex_);
    QShortcut _closeShortCut;
    void addStreamSelector(RtspPlayerWidget* widget_, int position_);
    
    Ui::SecondaryWindow* _ui;
    
    std::shared_ptr<rclcpp::Node> _node;

    struct PredefinedStream {
        QString name;
        QString url;
    };
    std::vector<PredefinedStream> _predefinedStreams;
    
    struct ActiveStream {
        std::unique_ptr<RtspPlayerWidget> widget;
        std::unique_ptr<QLabel> headerLabel;
        int predefinedStreamIndex = -1; 
        bool isRunning = false;
    };
    std::vector<ActiveStream> _activeStreams;
    
    CameraSettings* _cameraSettings;
    
    static constexpr int MAX_STREAMS = 6;
    int _currentStreamIndex = 0;
};

#endif  