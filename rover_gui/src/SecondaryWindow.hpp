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
    void addStream();
    void editStream();
    void removeStream();
    void onStreamSelection(int index);
    void onStreamStateChanged(bool running, int streamIndex);

private:
    QWidget _centralWidget;
    QVBoxLayout* _mainLayout;
    
    // Layout widgets
    QStackedWidget* _layoutStack;  // Holds different layout types
    QWidget* _singleStreamView;    // Container for single stream mode
    QWidget* _multiStreamView;     // Container for multi-stream mode
    
    QStackedWidget* _singleStreamStack; // For switching between streams in single mode
    QGridLayout* _multiStreamGrid;      // Grid for 2 or 4 stream mode
    
    // Layout controls
    QHBoxLayout* _controlLayout;
    QPushButton* _addStreamBtn;
    QPushButton* _removeStreamBtn;
    QPushButton* _editStreamBtn;
    QComboBox* _streamSelector;
    QComboBox* _layoutSelector;
    
    // Stream list and properties
    struct StreamInfo {
        RtspPlayerWidget* widget;
        QLabel* headerLabel;
        QString name;
        QString url;
        bool isRunning;
    };
    std::vector<StreamInfo> _streams;
    int _maxStreams = 4;
    int _currentStreamIndex = 0;
    
    void setupLayout();
    void updateLayout();
    void setupSingleStreamView();
    void setupMultiStreamView();
    void updateStreamSelectionCombo();
    void updateStreamHeader(int streamIndex);
    void updateAllStreamHeaders();
    QString generateDefaultStreamName(int index);
    void selectStream(int streamIndex);
};

#endif  // SECONDARY_WINDOW_HPP