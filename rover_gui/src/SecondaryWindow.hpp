#ifndef SECONDARY_WINDOW_HPP
#define SECONDARY_WINDOW_HPP

#include <QMainWindow>
#include <QGridLayout>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QStackedWidget>
#include <QMap>
#include <QLineEdit>
#include <QDialog>
#include <vector>
#include "QRtspPlayer/QRtspPlayerWidget.hpp"
#include "QRtspPlayer/StreamDialog.hpp"

class SecondaryWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit SecondaryWindow();
    ~SecondaryWindow();

private slots:
    void onLayoutChange(int index);
    void onStreamStateChanged(bool running, int streamIndex);

private:
    QWidget _centralWidget;
    QVBoxLayout* _mainLayout;
    
    // Layout widgets
    QStackedWidget* _layoutStack;  // Holds different layout types
    QWidget* _singleStreamView;    // Container for single stream mode
    QWidget* _multiStreamView;     // Container for multi-stream mode
    
    QStackedWidget* _singleStreamStack; // For switching between streams in single mode
    QGridLayout* _multiStreamGrid;      // Grid for 2, 4 or 6 stream mode
    
    // Layout controls
    QHBoxLayout* _controlLayout;
    QComboBox* _layoutSelector;
    
    // Predefined streams
    struct PredefinedStream {
        QString name;
        QString url;
    };
    std::vector<PredefinedStream> _predefinedStreams;
    
    // Active streams
    struct ActiveStream {
        RtspPlayerWidget* widget;
        QLabel* headerLabel;
        int predefinedStreamIndex; // Index in the predefined streams list
        bool isRunning;
    };
    std::vector<ActiveStream> _activeStreams;
    int _maxStreams = 6;
    int _currentStreamIndex = 0;
    
    void setupUI();
    void updateLayout();
    void setupSingleStreamView();
    void setupMultiStreamView();
    void updateStreamHeader(int streamIndex);
    void updateAllStreamHeaders();
    void loadPredefinedStreams(); // Method to load predefined streams
    void addStreamSelector(RtspPlayerWidget* widget, int position);
};

#endif  // SECONDARY_WINDOW_HPP