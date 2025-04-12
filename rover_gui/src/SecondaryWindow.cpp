#include "SecondaryWindow.hpp"
#include <QLabel>
#include <QGroupBox>

SecondaryWindow::SecondaryWindow():
    QMainWindow(nullptr),
    _centralWidget(this),
    _currentStreamIndex(0)
{
    // Setup main layout
    _mainLayout = new QVBoxLayout(&_centralWidget);
    
    // Load predefined streams first
    loadPredefinedStreams();
    
    // Create layout stack (to switch between single/multi views)
    _layoutStack = new QStackedWidget();
    
    // Setup single and multi stream views
    setupSingleStreamView();
    setupMultiStreamView();
    
    // Add views to layout stack
    _layoutStack->addWidget(_singleStreamView);
    _layoutStack->addWidget(_multiStreamView);
    
    // Initialize active streams with empty placeholders
    for (int i = 0; i < _maxStreams; i++) {
        _activeStreams.push_back({
            new RtspPlayerWidget(nullptr, QString("stream_%1").arg(i)),
            new QLabel(""),
            -1, // No predefined stream selected
            false
        });
    }
    
    // Setup the rest of the UI
    setupUI();
    
    // Update all stream headers
    updateAllStreamHeaders();
    
    // Update the layout
    updateLayout();
}

SecondaryWindow::~SecondaryWindow()
{
    // Clean up widgets
    for (auto& streamInfo : _activeStreams) {
        delete streamInfo.widget;
        delete streamInfo.headerLabel;
    }
}

void SecondaryWindow::loadPredefinedStreams()
{
    // Add some sample predefined streams
    // In a real application, these might be loaded from a configuration file
    _predefinedStreams = {
        {"Front Camera", "rtsp://example.com/front"},
        {"Back Camera", "rtsp://example.com/back"},
        {"Side Camera", "rtsp://example.com/side"},
        {"Overhead Camera", "rtsp://example.com/overhead"},
        {"Left Camera", "rtsp://example.com/left"},
        {"Right Camera", "rtsp://example.com/right"}
    };
}

void SecondaryWindow::setupUI() 
{
    // Create control layout
    _controlLayout = new QHBoxLayout();
    
    // Create layout selector
    QLabel* layoutLabel = new QLabel("Layout:");
    _layoutSelector = new QComboBox();
    _layoutSelector->addItem("Single Stream");
    _layoutSelector->addItem("2 Streams");
    _layoutSelector->addItem("4 Streams");
    _layoutSelector->addItem("6 Streams");
    
    // Add widgets to control layout
    _controlLayout->addWidget(layoutLabel);
    _controlLayout->addWidget(_layoutSelector);
    _controlLayout->addStretch();
    
    // Add layouts to main layout
    _mainLayout->addLayout(_controlLayout);
    _mainLayout->addWidget(_layoutStack, 1); // Give the layout stack a stretch factor
    
    // Set central widget
    this->setCentralWidget(&_centralWidget);
    
    // Set main window title
    this->setWindowTitle("Camera Streams");
    
    // Connect signals
    connect(_layoutSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &SecondaryWindow::onLayoutChange);
    
    // Connect signals for active streams and add selectors
    for (int i = 0; i < static_cast<int>(_activeStreams.size()); i++) {
        connect(_activeStreams[i].widget, &RtspPlayerWidget::streamStateChanged,
                this, [this, i](bool running, int) { 
                    this->onStreamStateChanged(running, static_cast<int>(i)); 
                });
                
        // Add stream selector to each widget
        addStreamSelector(_activeStreams[i].widget, i);
    }
}

void SecondaryWindow::addStreamSelector(RtspPlayerWidget* widget, int position)
{
    // Find the top layout in the widget
    QHBoxLayout* topLayout = widget->findChild<QHBoxLayout*>("topLayout");
    if (!topLayout) return;
    
    // Find the play/pause button
    QWidget* playPauseBtn = widget->findChild<QWidget*>("playPauseButton");
    if (!playPauseBtn) return;
    
    // Create the stream selector
    QComboBox* selector = new QComboBox(widget);
    selector->setObjectName(QString("streamSelector_%1").arg(position));
    selector->setMaximumWidth(150);
    
    // Add "None" option and all predefined streams
    selector->addItem("None");
    for (const auto& stream : _predefinedStreams) {
        selector->addItem(stream.name);
    }
    
    // Insert selector to the left of play/pause button
    int btnIndex = topLayout->indexOf(playPauseBtn);
    if (btnIndex >= 0) {
        topLayout->insertWidget(btnIndex, selector);
    }
    
    // Connect signal
    connect(selector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, position](int index) {
                // Check if this is a valid position and selection
                if (position < 0 || position >= _maxStreams) {
                    return;
                }
                
                if (index <= 0) {
                    // "None" selected - clear the stream
                    _activeStreams[position].predefinedStreamIndex = -1;
                    _activeStreams[position].widget->stopStream();
                    _activeStreams[position].headerLabel->setText("");
                } else if (index <= static_cast<int>(_predefinedStreams.size())) {
                    // A predefined stream was selected
                    int predefinedIndex = index - 1; // -1 because index 0 is "None"
                    
                    // Update active stream info
                    _activeStreams[position].predefinedStreamIndex = predefinedIndex;
                    
                    // Find the URL input field by name
                    QLineEdit* urlInput = _activeStreams[position].widget->findChild<QLineEdit*>("rtspUrlInput");
                    if (urlInput) {
                        // Set URL text
                        urlInput->setText(_predefinedStreams[predefinedIndex].url);
                    }
                    
                    // Update header
                    updateStreamHeader(position);
                    
                    // Start the stream - only here, don't rely on URL change events
                    // FIXED: Let the play button handle starting the stream instead of doing it here
                    // This avoids the duplicate "Starting stream" log messages
                }
            });
}

void SecondaryWindow::setupSingleStreamView()
{
    _singleStreamView = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(_singleStreamView);
    layout->setContentsMargins(0, 0, 0, 0);
    
    // Create stacked widget for streams
    _singleStreamStack = new QStackedWidget();
    layout->addWidget(_singleStreamStack);
}

void SecondaryWindow::setupMultiStreamView()
{
    _multiStreamView = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(_multiStreamView);
    layout->setContentsMargins(0, 0, 0, 0);
    
    // Create grid layout for streams
    _multiStreamGrid = new QGridLayout();
    _multiStreamGrid->setSpacing(4);
    layout->addLayout(_multiStreamGrid);
}

void SecondaryWindow::onLayoutChange(int index)
{
    // Update the layout
    updateLayout();
}

void SecondaryWindow::updateLayout()
{
    // Determine layout mode
    int layoutMode = _layoutSelector->currentIndex();
    
    // Clear layouts first
    while (_singleStreamStack->count() > 0) {
        QWidget* widget = _singleStreamStack->widget(0);
        _singleStreamStack->removeWidget(widget);
    }
    
    while (_multiStreamGrid->count() > 0) {
        QLayoutItem* item = _multiStreamGrid->takeAt(0);
        if (item->widget()) {
            item->widget()->setParent(nullptr);
        }
        delete item;
    }
    
    // Determine number of streams for this layout
    int numStreams = 0;
    switch (layoutMode) {
        case 0: numStreams = 1; break; // Single stream
        case 1: numStreams = 2; break; // 2 streams
        case 2: numStreams = 4; break; // 4 streams
        case 3: numStreams = 6; break; // 6 streams
    }
    
    // Update layouts based on mode
    switch(layoutMode) {
        case 0: // Single Stream
        {
            // Add single stream to stacked widget
            for (int i = 0; i < numStreams; i++) {
                // Create a container for header and widget
                QWidget* container = new QWidget();
                QVBoxLayout* containerLayout = new QVBoxLayout(container);
                containerLayout->setContentsMargins(0, 0, 0, 0);
                
                // Add header and widget to container
                containerLayout->addWidget(_activeStreams[i].headerLabel);
                containerLayout->addWidget(_activeStreams[i].widget);
                
                // Add container to stacked widget
                _singleStreamStack->addWidget(container);
            }
            
            // Show the current stream
            if (_singleStreamStack->count() > 0) {
                _singleStreamStack->setCurrentIndex(0);
            }
            
            // Show single stream view
            _layoutStack->setCurrentWidget(_singleStreamView);
            break;
        }
            
        case 1: // 2 Streams
        case 2: // 4 Streams
        case 3: // 6 Streams
        {
            int rows = (layoutMode == 1) ? 2 : (layoutMode == 2) ? 2 : 3;
            int cols = 2;
            
            for (int i = 0; i < numStreams; i++) {
                // Create a container for header and widget
                QWidget* container = new QWidget();
                QVBoxLayout* containerLayout = new QVBoxLayout(container);
                containerLayout->setContentsMargins(0, 0, 0, 0);
                containerLayout->setSpacing(0);
                
                // Add header and widget to container
                containerLayout->addWidget(_activeStreams[i].headerLabel);
                containerLayout->addWidget(_activeStreams[i].widget);
                
                // Add container to grid layout
                int row = i / cols;
                int col = i % cols;
                _multiStreamGrid->addWidget(container, row, col);
            }
            _layoutStack->setCurrentWidget(_multiStreamView);
            break;
        }
    }
}

void SecondaryWindow::updateStreamHeader(int streamIndex)
{
    if (streamIndex >= 0 && streamIndex < static_cast<int>(_activeStreams.size())) {
        auto& stream = _activeStreams[streamIndex];
        
        if (stream.predefinedStreamIndex >= 0 && 
            stream.predefinedStreamIndex < static_cast<int>(_predefinedStreams.size())) {
            // Create header text with name and URL
            const auto& predefined = _predefinedStreams[stream.predefinedStreamIndex];
            QString headerText = predefined.name;
            if (!predefined.url.isEmpty()) {
                headerText += " - " + predefined.url;
            }
            
            // Update header label
            stream.headerLabel->setText(headerText);
            stream.headerLabel->setFrameShape(QFrame::StyledPanel);
            stream.headerLabel->setFrameShadow(QFrame::Raised);
            stream.headerLabel->setAlignment(Qt::AlignCenter);
        }
    }
}

void SecondaryWindow::updateAllStreamHeaders()
{
    for (size_t i = 0; i < _activeStreams.size(); i++) {
        updateStreamHeader(static_cast<int>(i));
    }
}

void SecondaryWindow::onStreamStateChanged(bool running, int streamIndex)
{
    if (streamIndex >= 0 && streamIndex < static_cast<int>(_activeStreams.size())) {
        _activeStreams[streamIndex].isRunning = running;
    }
}