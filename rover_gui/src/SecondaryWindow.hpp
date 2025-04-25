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
    explicit SecondaryWindow(std::shared_ptr<rclcpp::Node> node = nullptr);
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
    void initializeRosServicesForWidgets(void);
    
    // Stream container helpers
    QWidget* createStreamContainer(int streamIndex_);
    void setupSingleStreamView(void);
    void setupMultiStreamView(void);
    void addStreamSelector(RtspPlayerWidget* widget_, int position_);
    
    // Core widget components - now stack allocated
    QWidget _centralWidget;
    QVBoxLayout _mainLayout;
    QStackedWidget _layoutStack;
    QWidget _singleStreamView;
    QWidget _multiStreamView;
    
    // ROS components
    std::shared_ptr<rclcpp::Node> _node;

    // Layout components - now stack allocated
    QStackedWidget _singleStreamStack;
    QGridLayout _multiStreamGrid;
    QHBoxLayout _controlLayout;
    QComboBox _layoutSelector;
    
    // Stream configuration
    struct PredefinedStream {
        QString name;
        QString url;
    };
    std::vector<PredefinedStream> _predefinedStreams;
    
    // Active stream management
    struct ActiveStream {
        // Note: Using raw pointers here as these are special widgets
        // that still need to be heap allocated due to how they're used
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