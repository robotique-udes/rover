#ifndef SECONDARY_WINDOW_HPP
#define SECONDARY_WINDOW_HPP

#include "QVideoPlayer/QVideoManagerWidget.hpp"
#include <QMainWindow>
#include <QGridLayout>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QStackedWidget>
#include <QMap>
#include <QLineEdit>
#include <QDialog>
#include <memory>
#include <vector>
#include "QRtspPlayer/QRtspPlayerWidgetHeader.hpp"

class SecondaryWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit SecondaryWindow();
    ~SecondaryWindow();

private slots:
    void onLayoutChange(int index_);
    void onStreamStateChanged(bool running_, int streamIndex_);

private:
    // Layout types
    enum class LayoutMode {
        SingleStream = 0,
        TwoStreams = 1,
        FourStreams = 2,
        SixStreams = 3
    };
    
    // UI setup methods
    void setupUI(void);
    void loadPredefinedStreams(void);
    void initializeStreams(void);
    void updateLayout(void);
    
    // Stream container helpers
    QWidget* createStreamContainer(int streamIndex_);
    void setupSingleStreamView(void);
    void setupMultiStreamView(void);
    void addStreamSelector(RtspPlayerWidget* widget_, int position_);
    
    // Core widget components
    QWidget _centralWidget;
    QVBoxLayout* _mainLayout = nullptr;
    QStackedWidget* _layoutStack = nullptr;
    QWidget* _singleStreamView = nullptr;
    QWidget* _multiStreamView = nullptr;
    
    // ROS components
    std::shared_ptr<rclcpp::Node> _node;
    std::shared_ptr<rclcpp::Client<rover_msgs::srv::ArucoDetection>> _arucoDetectionClient;

    // Layout components
    QStackedWidget* _singleStreamStack = nullptr;
    QGridLayout* _multiStreamGrid = nullptr;
    QHBoxLayout* _controlLayout = nullptr;
    QComboBox* _layoutSelector = nullptr;
    
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
    
    // Configuration
    static constexpr int MAX_STREAMS = 6;
    int _currentStreamIndex = 0;
};

#endif  // SECONDARY_WINDOW_HPP