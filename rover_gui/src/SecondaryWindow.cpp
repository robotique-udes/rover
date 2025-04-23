#include "SecondaryWindow.hpp"
#include <QLabel>
#include <QGroupBox>

SecondaryWindow::SecondaryWindow():
    QMainWindow(nullptr),
    _centralWidget(this),
    _currentStreamIndex(0)
{
    // Initialize ROS node
    _node = std::make_shared<rclcpp::Node>("secondary_window_node");
    
    // Create Aruco detection client
    _arucoDetectionClient = _node->create_client<rover_msgs::srv::ArucoDetection>("/rover/auxiliary/aruco/manager");
    
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
    for (int i = 0; i < MAX_STREAMS; i++) {
        ActiveStream stream;
        stream.widget = std::make_unique<RtspPlayerWidget>(nullptr, QString("stream_%1").arg(i));
        stream.predefinedStreamIndex = -1; // No predefined stream selected
        stream.isRunning = false;
        _activeStreams.push_back(std::move(stream));
    }

    // Set the Aruco manager for each widget
    for (auto& streamInfo : _activeStreams) {
        streamInfo.widget->setArucoDetectionManager(_arucoDetectionClient);
    }
    
    // Setup the rest of the UI
    setupUI();
    
    // Update the layout
    updateLayout();
}

SecondaryWindow::~SecondaryWindow()
{
    // Smart pointers will automatically clean up, no manual deletion needed
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
    // Create control layout with minimal margins
    _controlLayout = new QHBoxLayout();
    _controlLayout->setContentsMargins(3, 0, 3, 0); // Reduced vertical margins
    _controlLayout->setSpacing(2); // Minimal spacing
    
    // Create layout selector
    QLabel* layoutLabel = new QLabel("Layout:");
    _layoutSelector = new QComboBox();
    _layoutSelector->addItem("Single Stream");
    _layoutSelector->addItem("2 Streams");
    _layoutSelector->addItem("4 Streams");
    _layoutSelector->addItem("6 Streams");
    
    // Apply consistent styling to layout selector
    _layoutSelector->setFixedHeight(26);
    _layoutSelector->setStyleSheet("QComboBox { border: 1px solid #777777; border-radius: 2px; padding: 0px 2px; }");
    
    // Add widgets to control layout
    _controlLayout->addWidget(layoutLabel);
    _controlLayout->addWidget(_layoutSelector);
    _controlLayout->addStretch();
    
    // Add layouts to main layout with minimal spacing
    _mainLayout->setContentsMargins(0, 0, 0, 0);
    _mainLayout->setSpacing(0); // Zero spacing to maximize video area
    _mainLayout->addLayout(_controlLayout);
    _mainLayout->addWidget(_layoutStack, 1);
    
    // Set central widget
    this->setCentralWidget(&_centralWidget);
    
    // Set main window title
    this->setWindowTitle("Camera Streams");
    
    // Connect signals
    connect(_layoutSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &SecondaryWindow::onLayoutChange);
    
    // Connect signals for active streams and add selectors
    for (int i = 0; i < static_cast<int>(_activeStreams.size()); i++) {
        connect(_activeStreams[i].widget.get(), &RtspPlayerWidget::streamStateChanged,
                this, [this, i](bool running, int) { 
                    this->onStreamStateChanged(running, static_cast<int>(i)); 
                });
                
        // Add stream selector to each widget
        addStreamSelector(_activeStreams[i].widget.get(), i);
    }
}

void SecondaryWindow::addStreamSelector(RtspPlayerWidget* widget, int position)
{
    // Find the existing stream selector in the widget (from the UI file)
    QComboBox* selector = widget->findChild<QComboBox*>("streamSelector");
    if (!selector) {
        // If not found (shouldn't happen if it's in the UI file), log a warning and return
        qDebug() << "Warning: Could not find streamSelector in widget" << position;
        return;
    }
    
    // Clear any existing items (important if we're reusing the widget)
    selector->clear();
    
    // Add "None" option and all predefined streams
    selector->addItem("None");
    for (const auto& stream : _predefinedStreams) {
        selector->addItem(stream.name);
    }
    
    // Connect signal for selection changes
    // First disconnect any existing connections to avoid duplicates
    disconnect(selector, QOverload<int>::of(&QComboBox::currentIndexChanged), nullptr, nullptr);
    
    // Now connect the signal
    connect(selector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, position](int index) {
                // Check if this is a valid position and selection
                if (position < 0 || position >= MAX_STREAMS) {
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
                }
            });
}
void SecondaryWindow::setupSingleStreamView()
{
    _singleStreamView = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(_singleStreamView);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0); // Zero spacing
    
    // Create stacked widget for streams
    _singleStreamStack = new QStackedWidget();
    layout->addWidget(_singleStreamStack);
}

void SecondaryWindow::setupMultiStreamView()
{
    _multiStreamView = new QWidget();
    QVBoxLayout* layout = new QVBoxLayout(_multiStreamView);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0); // Zero spacing
    
    // Create grid layout for streams
    _multiStreamGrid = new QGridLayout();
    _multiStreamGrid->setSpacing(1); // Minimal spacing between streams
    layout->addLayout(_multiStreamGrid);
}

void SecondaryWindow::onLayoutChange(int /* index_ */)
{
    // Update the layout (index parameter not used directly but needed for signal connection)
    updateLayout();
}

void SecondaryWindow::updateLayout()
{
    // Determine layout mode
    int layoutMode = _layoutSelector->currentIndex();
    _multiStreamGrid->setSpacing(1);

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
                
                // Add header and widget to container - use .get() to get raw pointers
                containerLayout->addWidget(_activeStreams[i].headerLabel.get());
                containerLayout->addWidget(_activeStreams[i].widget.get());
                
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
            // Fixed: rows variable is used, but we compute it directly
            int cols = 2;
            
            for (int i = 0; i < numStreams; i++) {
                // Create a container for header and widget
                QWidget* container = new QWidget();
                QVBoxLayout* containerLayout = new QVBoxLayout(container);
                containerLayout->setContentsMargins(0, 0, 0, 0);
                containerLayout->setSpacing(0);
                
                // Add header and widget to container - use .get() to get raw pointers
                containerLayout->addWidget(_activeStreams[i].widget.get());
                
                // Add container to grid layout
                int row = i / cols;
                int col = i % cols;
                _multiStreamGrid->addWidget(_activeStreams[i].widget.get(), row, col);
            }
            _layoutStack->setCurrentWidget(_multiStreamView);
            break;
        }
    }
}

void SecondaryWindow::onStreamStateChanged(bool running, int streamIndex)
{
    if (streamIndex >= 0 && streamIndex < static_cast<int>(_activeStreams.size())) {
        _activeStreams[streamIndex].isRunning = running;
    }
}