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
    
signals:
    void requestStartStream(const QString& rtspUrl);
    void requestStopStream();
    void streamStateChanged(bool isRunning, int streamIndex);

private:
    static int instanceCounter;
    QString _widgetId;
    int _streamIndex;

    Ui::RtspPlayerWidget* ui;

    QThread* workerThread;
    GStreamerWorker* gstreamerWorker;

    QTimer* reconnectTimer;
    QTimer* frameTimeoutTimer;

    GstElement* pipeline;
    bool receivingFrames;
    bool inReconnectionMode;
    void updateUrlValidationUI(bool isValid);
    
    bool validateRtspUrl(QString& url);
    QString correctRtspUrl(const QString& url);

    // Main stacked widget to switch between views
    QStackedWidget* _stackedWidget;
    
    // Main video view
    QWidget* _videoWidget;
    
    // Our custom play/pause button
    QPlayPauseButton* _playPauseButton;
    
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

    void updateStatusIndicator(const QString& color);
    void setupUI();
    void emitStateChanged();
};

#endif