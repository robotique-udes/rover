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
    
    // Create layout stack (to switch between single/multi views)
    _layoutStack = new QStackedWidget();
    
    // Setup single and multi stream views
    setupSingleStreamView();
    setupMultiStreamView();
    
    // Add views to layout stack
    _layoutStack->addWidget(_singleStreamView);
    _layoutStack->addWidget(_multiStreamView);
    
    // Create control layout
    _controlLayout = new QHBoxLayout();
    
    // Create layout selector
    QLabel* layoutLabel = new QLabel("Layout:");
    _layoutSelector = new QComboBox();
    _layoutSelector->addItem("Single Stream");
    _layoutSelector->addItem("2 Streams");
    _layoutSelector->addItem("4 Streams");
    _layoutSelector->addItem("6 Streams");
    
    // Create stream selection combo
    _streamSelector = new QComboBox();
    
    // Create stream management buttons
    _addStreamBtn = new QPushButton("Add Stream");
    _editStreamBtn = new QPushButton("Edit Stream");
    _removeStreamBtn = new QPushButton("Remove Stream");
    
    // Add widgets to control layout
    _controlLayout->addWidget(layoutLabel);
    _controlLayout->addWidget(_layoutSelector);
    _controlLayout->addWidget(new QLabel("Stream:"));
    _controlLayout->addWidget(_streamSelector);
    _controlLayout->addStretch();
    _controlLayout->addWidget(_addStreamBtn);
    _controlLayout->addWidget(_editStreamBtn);
    _controlLayout->addWidget(_removeStreamBtn);
    
    // Add layouts to main layout
    _mainLayout->addLayout(_controlLayout);
    _mainLayout->addWidget(_layoutStack, 1); // Give the layout stack a stretch factor
    
    // Set central widget
    this->setCentralWidget(&_centralWidget);
    
    // Start with no streams initially
    
    // Set main window title
    this->setWindowTitle("Camera Streams");
    
    // Connect signals
    connect(_layoutSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &SecondaryWindow::onLayoutChange);
    connect(_streamSelector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &SecondaryWindow::onStreamSelection);
    connect(_addStreamBtn, &QPushButton::clicked, this, &SecondaryWindow::addStream);
    connect(_editStreamBtn, &QPushButton::clicked, this, &SecondaryWindow::editStream);
    connect(_removeStreamBtn, &QPushButton::clicked, this, &SecondaryWindow::removeStream);
    
    // Update the UI
    updateStreamSelectionCombo();
    updateAllStreamHeaders();
    updateLayout();
}

SecondaryWindow::~SecondaryWindow()
{
    // Clean up widgets
    for (auto& streamInfo : _streams) {
        delete streamInfo.widget;
        delete streamInfo.headerLabel;
    }
}

QString SecondaryWindow::generateDefaultStreamName(int index)
{
    return QString("Stream %1").arg(index + 1);
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
    
    // Update layouts based on mode
    switch(layoutMode) {
        case 0: // Single Stream
        {
            // Add all stream widgets to stacked widget with their headers
            for (auto& streamInfo : _streams) {
                // Create a container for header and widget
                QWidget* container = new QWidget();
                QVBoxLayout* containerLayout = new QVBoxLayout(container);
                containerLayout->setContentsMargins(0, 0, 0, 0);
                
                // Add header and widget to container
                containerLayout->addWidget(streamInfo.headerLabel);
                containerLayout->addWidget(streamInfo.widget);
                
                // Add container to stacked widget
                _singleStreamStack->addWidget(container);
            }
            
            // Select current stream
            if (_singleStreamStack->count() > 0) {
                _singleStreamStack->setCurrentIndex(_currentStreamIndex);
            }
            
            // Show single stream view
            _layoutStack->setCurrentWidget(_singleStreamView);
            break;
        }
            
        case 1: // 2 Streams
        {
            // Add up to 2 streams in vertical layout with their headers
            for (size_t i = 0; i < _streams.size() && i < 2; i++) {
                // Create a container for header and widget
                QWidget* container = new QWidget();
                QVBoxLayout* containerLayout = new QVBoxLayout(container);
                containerLayout->setContentsMargins(0, 0, 0, 0);
                
                // Add header and widget to container
                containerLayout->addWidget(_streams[i].headerLabel);
                containerLayout->addWidget(_streams[i].widget);
                
                // Add container to grid layout
                _multiStreamGrid->addWidget(container, static_cast<int>(i), 0);
            }
            _layoutStack->setCurrentWidget(_multiStreamView);
            break;
        }
            
        case 2: // 4 Streams
        {
            // Special case for exactly 3 streams - make the third stream take the full width
            if (_streams.size() == 3) {
                // First two streams in top row
                for (size_t i = 0; i < 2; i++) {
                    // Create a container for header and widget
                    QWidget* container = new QWidget();
                    QVBoxLayout* containerLayout = new QVBoxLayout(container);
                    containerLayout->setContentsMargins(0, 0, 0, 0);
                    containerLayout->setSpacing(0);
                    
                    // Add header and widget to container
                    containerLayout->addWidget(_streams[i].headerLabel);
                    containerLayout->addWidget(_streams[i].widget);
                    
                    // Add container to grid layout - top row
                    _multiStreamGrid->addWidget(container, 0, static_cast<int>(i));
                }
                
                // Third stream spans full width on bottom row
                QWidget* container = new QWidget();
                QVBoxLayout* containerLayout = new QVBoxLayout(container);
                containerLayout->setContentsMargins(0, 0, 0, 0);
                containerLayout->setSpacing(0);
                
                // Add header and widget to container
                containerLayout->addWidget(_streams[2].headerLabel);
                containerLayout->addWidget(_streams[2].widget);
                
                // Add container to grid layout - bottom row, spanning two columns
                _multiStreamGrid->addWidget(container, 1, 0, 1, 2);
            }
            // Normal case - up to 4 streams in 2x2 grid
            else {
                for (size_t i = 0; i < _streams.size() && i < 4; i++) {
                    // Create a container for header and widget
                    QWidget* container = new QWidget();
                    QVBoxLayout* containerLayout = new QVBoxLayout(container);
                    containerLayout->setContentsMargins(0, 0, 0, 0);
                    containerLayout->setSpacing(0);  // Minimize spacing
                    
                    // Add header and widget to container
                    containerLayout->addWidget(_streams[i].headerLabel);
                    containerLayout->addWidget(_streams[i].widget);
                    
                    // Add container to grid layout
                    int row = static_cast<int>(i) / 2;
                    int col = static_cast<int>(i) % 2;
                    _multiStreamGrid->addWidget(container, row, col);
                }
            }
            _layoutStack->setCurrentWidget(_multiStreamView);
            break;
        }
        
        case 3: // 6 Streams (new case)
        {
            // Special case for exactly 5 streams - make the fifth stream take the full width of the bottom row
            if (_streams.size() == 5) {
                // First four streams in a 2x2 grid
                for (size_t i = 0; i < 4; i++) {
                    // Create a container for header and widget
                    QWidget* container = new QWidget();
                    QVBoxLayout* containerLayout = new QVBoxLayout(container);
                    containerLayout->setContentsMargins(0, 0, 0, 0);
                    containerLayout->setSpacing(0);
                    
                    // Add header and widget to container
                    containerLayout->addWidget(_streams[i].headerLabel);
                    containerLayout->addWidget(_streams[i].widget);
                    
                    // Add container to grid layout - 2x2 grid for first 4 streams
                    int row = static_cast<int>(i) / 2;
                    int col = static_cast<int>(i) % 2;
                    _multiStreamGrid->addWidget(container, row, col);
                }
                
                // Fifth stream spans full width on bottom row
                QWidget* container = new QWidget();
                QVBoxLayout* containerLayout = new QVBoxLayout(container);
                containerLayout->setContentsMargins(0, 0, 0, 0);
                containerLayout->setSpacing(0);
                
                // Add header and widget to container
                containerLayout->addWidget(_streams[4].headerLabel);
                containerLayout->addWidget(_streams[4].widget);
                
                // Add container to grid layout - bottom row, spanning two columns
                _multiStreamGrid->addWidget(container, 2, 0, 1, 2);
            }
            // Normal case - up to 6 streams in 3x2 grid
            else {
                for (size_t i = 0; i < _streams.size() && i < 6; i++) {
                    // Create a container for header and widget
                    QWidget* container = new QWidget();
                    QVBoxLayout* containerLayout = new QVBoxLayout(container);
                    containerLayout->setContentsMargins(0, 0, 0, 0);
                    containerLayout->setSpacing(0);
                    
                    // Add header and widget to container
                    containerLayout->addWidget(_streams[i].headerLabel);
                    containerLayout->addWidget(_streams[i].widget);
                    
                    // Add container to grid layout - 3 rows, 2 columns
                    int row = static_cast<int>(i) / 2;
                    int col = static_cast<int>(i) % 2;
                    _multiStreamGrid->addWidget(container, row, col);
                }
            }
            _layoutStack->setCurrentWidget(_multiStreamView);
            break;
        }
    }
    
    // Update button state
    _addStreamBtn->setEnabled(static_cast<int>(_streams.size()) < _maxStreams);
    _removeStreamBtn->setEnabled(_streams.size() > 1);
    _editStreamBtn->setEnabled(!_streams.empty());
}

void SecondaryWindow::updateStreamHeader(int streamIndex)
{
    if (streamIndex >= 0 && streamIndex < static_cast<int>(_streams.size())) {
        auto& stream = _streams[streamIndex];
        
        // Create header text with name and URL
        QString headerText = stream.name;
        if (!stream.url.isEmpty()) {
            headerText += " - " + stream.url;
        }
        
        // Update header label
        stream.headerLabel->setText(headerText);
        stream.headerLabel->setFrameShape(QFrame::StyledPanel);
        stream.headerLabel->setFrameShadow(QFrame::Raised);
        stream.headerLabel->setAlignment(Qt::AlignCenter);
        
        // Find the URL input field by name (using findChild works with both pointer and stack-based ui)
        QLineEdit* urlInput = stream.widget->findChild<QLineEdit*>("rtspUrlInput");
        if (urlInput && !stream.url.isEmpty()) {
            // Set URL text but don't trigger validation (just set the text)
            urlInput->blockSignals(true);
            urlInput->setText(stream.url);
            urlInput->blockSignals(false);
            
            // Manually trigger validation to update the color
            bool isValid = stream.widget->validateRtspUrl(stream.url);
            stream.widget->updateUrlValidationUI(isValid);
        }
    }
}

void SecondaryWindow::updateAllStreamHeaders()
{
    for (size_t i = 0; i < _streams.size(); i++) {
        updateStreamHeader(static_cast<int>(i));
    }
}

void SecondaryWindow::updateStreamSelectionCombo()
{
    // Save current selection
    int previousIndex = _streamSelector->currentIndex();
    if (previousIndex < 0) {
        previousIndex = 0;
    }
    
    // Update stream selection combo without triggering signals
    _streamSelector->blockSignals(true);
    _streamSelector->clear();
    
    for (const auto& streamInfo : _streams) {
        // Just show the stream name (no status)
        _streamSelector->addItem(streamInfo.name);
    }
    
    // Restore selection or select first item if empty
    if (_streamSelector->count() > 0) {
        // Make sure we don't exceed the bounds
        int newIndex = qMin(previousIndex, _streamSelector->count() - 1);
        _streamSelector->setCurrentIndex(newIndex);
        _currentStreamIndex = newIndex;
    }
    
    _streamSelector->blockSignals(false);
}

void SecondaryWindow::onLayoutChange(int index)
{
    // Simply update the layout without forcing stream creation
    // No need to check for required widgets or auto-create streams
    
    // Update the UI
    Q_UNUSED(index);
    updateStreamSelectionCombo();
    updateLayout();
}

void SecondaryWindow::addStream()
{
    if (static_cast<int>(_streams.size()) < _maxStreams) {
        int newIdx = static_cast<int>(_streams.size());
        
        // Create a stream adding dialog
        StreamDialog dialog("Add Stream", generateDefaultStreamName(newIdx));
        
        if (dialog.exec() == QDialog::Accepted) {
            QString name = dialog.getStreamName().trimmed();
            QString url = dialog.getStreamUrl().trimmed();
            
            if (name.isEmpty()) {
                name = generateDefaultStreamName(newIdx);
            }
            
            // Create new stream
            _streams.push_back({
                new RtspPlayerWidget(nullptr),
                new QLabel(name),
                name,
                url,
                false
            });
            
            // Find the URL input and set it
            QLineEdit* urlInput = _streams.back().widget->findChild<QLineEdit*>("rtspUrlInput");
            if (urlInput && !url.isEmpty()) {
                urlInput->setText(url);
            }
            
            // Start the stream if URL is provided
            if (!url.isEmpty()) {
                _streams.back().widget->startStream(url);
            }
            
            // Update header for the new stream
            updateStreamHeader(newIdx);
            
            // Select the new stream
            _currentStreamIndex = newIdx;
            
            // Update the UI
            updateStreamSelectionCombo();
            updateLayout();
        }
    }
}

void SecondaryWindow::editStream()
{
    int index = _streamSelector->currentIndex();
    if (index >= 0 && index < static_cast<int>(_streams.size())) {
        auto& stream = _streams[index];
        
        // Create stream edit dialog
        StreamDialog dialog("Edit Stream", stream.name, stream.url);
        
        if (dialog.exec() == QDialog::Accepted) {
            QString name = dialog.getStreamName().trimmed();
            QString url = dialog.getStreamUrl().trimmed();
            
            if (name.isEmpty()) {
                name = generateDefaultStreamName(index);
            }
            
            // Update stream info
            stream.name = name;
            stream.url = url;
            
            // Stop current stream if running
            stream.widget->stopStream();
            
            // Update the URL in the widget's input field directly
            QLineEdit* urlInput = stream.widget->findChild<QLineEdit*>("rtspUrlInput");
            if (urlInput) {
                // Set URL text but block signals
                urlInput->blockSignals(true);
                urlInput->setText(url);
                urlInput->blockSignals(false);
                
                // Manually trigger validation to update the color
                bool isValid = stream.widget->validateRtspUrl(url);
                stream.widget->updateUrlValidationUI(isValid);
            }
            
            // Start with new URL if provided
            if (!url.isEmpty()) {
                stream.widget->startStream(url);
            }
            
            // Update header
            updateStreamHeader(index);
            
            // Update the UI
            updateStreamSelectionCombo();
            updateLayout();
        }
    }
}

void SecondaryWindow::removeStream()
{
    int index = _streamSelector->currentIndex();
    
    if (_streams.size() > 1 && index >= 0 && index < static_cast<int>(_streams.size())) {
        // Clean up widgets for the stream being removed
        delete _streams[index].widget;
        delete _streams[index].headerLabel;
        
        // Remove the stream from the vector
        _streams.erase(_streams.begin() + index);
        
        // Adjust the current index if needed
        if (_currentStreamIndex >= static_cast<int>(_streams.size())) {
            _currentStreamIndex = static_cast<int>(_streams.size()) - 1;
        }
        
        // Update the UI
        updateStreamSelectionCombo();
        updateLayout();
    }
}

void SecondaryWindow::onStreamSelection(int index)
{
    if (index >= 0 && index < static_cast<int>(_streams.size())) {
        _currentStreamIndex = index;
        selectStream(index);
    }
}

void SecondaryWindow::selectStream(int streamIndex)
{
    if (streamIndex >= 0 && streamIndex < static_cast<int>(_streams.size())) {
        // Update the current stream index
        _currentStreamIndex = streamIndex;
        
        // If we're in single stream mode, update the visible stream
        if (_layoutSelector->currentIndex() == 0 && streamIndex < _singleStreamStack->count()) {
            _singleStreamStack->setCurrentIndex(streamIndex);
        }
    }
}

void SecondaryWindow::onStreamStateChanged(bool running, int streamIndex)
{
    if (streamIndex >= 0 && streamIndex < static_cast<int>(_streams.size())) {
        _streams[streamIndex].isRunning = running;
    }
}