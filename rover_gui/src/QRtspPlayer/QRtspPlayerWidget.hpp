#ifndef RTSPPLAYERWIDGET_HPP
#define RTSPPLAYERWIDGET_HPP

#include <QThread>
#include <QTimer>
#include <QWidget>
#include <QTextEdit>
#include <QCheckBox>
#include <QPushButton>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QLabel>
#include <QDateTime>
#include <gst/gst.h>

#include "QGStreamerWorker.hpp"
#include "UI_Player.h"
#include "QLogManager.hpp"
#include "QPlayPauseButton.hpp" // Include our custom button

class RtspPlayerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RtspPlayerWidget(QWidget* parent = nullptr, const QString& widgetId = QString());
    ~RtspPlayerWidget();

    void startStream(const QString& rtspUrl);
    void stopStream();
    
    // Get the widget's unique ID
    QString getId() const { return _widgetId; }
    
    // Get streaming state
    bool isStreaming() const { return receivingFrames; }
    
    // Make these methods public so they can be called from SecondaryWindow
    bool validateRtspUrl(const QString& url);
    void updateUrlValidationUI(bool isValid);
    
    // Toggle control visibility
    void setControlsVisible(bool visible);
    bool areControlsVisible() const { return _controlsVisible; }

private slots:
    void onPipelineStarted(GstElement* pipeline);
    void onErrorOccurred(const QString& error);
    void onNewLogMessage(const QString& message, const QString& target);
    void onToggleDebug(bool checked);
    void onToggleInfo(bool checked);
    void onToggleWarning(bool checked);
    void onToggleError(bool checked);
    void onClearLogs();
    void onToggleView();
    void onToggleControls();
    
signals:
    void requestStartStream(const QString& rtspUrl);
    void requestStopStream();
    void streamStateChanged(bool isRunning, int streamIndex);
    void controlsVisibilityChanged(bool visible);

private:
    static int instanceCounter;
    QString _widgetId;
    int _streamIndex;

    // Changed from pointer to object
    Ui::RtspPlayerWidget ui;

    QThread* workerThread;
    GStreamerWorker* gstreamerWorker;

    QTimer* reconnectTimer;
    QTimer* frameTimeoutTimer;
    QTimer* connectionTimeoutTimer; // New timer for initial connection timeout

    // For preventing duplicate stream starts
    QString _lastStreamUrl;
    QDateTime _lastStreamTime;

    GstElement* pipeline;
    bool receivingFrames;
    bool inReconnectionMode;
    int reconnectAttempts; // Counter for reconnection attempts
    static const int maxReconnectAttempts = 3; // Maximum number of reconnection attempts
    bool wasEverConnected; // Track if we ever successfully connected
    bool connectionFailed; // Track if connection failed permanently
    bool _controlsVisible; // Track visibility of controls

    // Main stacked widget to switch between views
    QStackedWidget* _stackedWidget;
    
    // Main video view
    QWidget* _videoWidget;
    
    // Video/Status stacked widget
    QStackedWidget* _videoStack;
    QWidget* _statusPage;
    QLabel* _statusLabel;
    
    // Our custom play/pause button
    QPlayPauseButton* _playPauseButton;
    
    // Toggle button for controls
    QPushButton* _toggleControlsButton;
    
    // Controls container
    QWidget* _controlsContainer;
    
    // Log view components
    QWidget* _logWidget;
    QVBoxLayout* _logLayout;
    QHBoxLayout* _logControlLayout;
    QTextEdit* _logDisplay;
    QCheckBox* _debugCheckbox;
    QCheckBox* _infoCheckbox;
    QCheckBox* _warningCheckbox;
    QCheckBox* _errorCheckbox;
    QPushButton* _clearButton;
    QPushButton* _toggleViewButton;

    void updateStatusText(const QString& text);
    void setupUI();
    void emitStateChanged();
};

#endif