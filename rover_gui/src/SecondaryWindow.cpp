#include "SecondaryWindow.hpp"
#include "../../rover_shared_libs/rover_lib2/src/rover_lib2/helpers/constants.hpp"
#include "Global/Constant/Keybinding.hpp"
#include "UI_SecondaryWindow.h" 

#include <QLabel>
#include <QGroupBox>
#include <QStackedWidget>
#include <QCloseEvent>


SecondaryWindow::SecondaryWindow(std::shared_ptr<rclcpp::Node> guiNode_):
    _closeShortCut(Constants::Keybinding::CLOSE_APP, this),  
    _ui(new Ui::SecondaryWindow),                            
    _node(guiNode_),
    _cameraSettings(new CameraSettings(this)),
    _currentStreamIndex(0)
{
if (!_node) {
    _node = std::make_shared<rclcpp::Node>("secondary_window_node");
}

_ui->setupUi(this); 

loadPredefinedStreams();

for (int i = 0; i < MAX_STREAMS; i++) {
    ActiveStream stream;
    stream.widget = std::make_unique<RtspPlayerWidget>(nullptr, QString("stream_%1").arg(i));
    stream.headerLabel = std::make_unique<QLabel>();
    stream.predefinedStreamIndex = -1;
    stream.isRunning = false;
    _activeStreams.push_back(std::move(stream));
}

connect(this, &QWidget::destroyed, qApp, &QCoreApplication::quit);
connect(&_closeShortCut, &QShortcut::activated, this, &QWidget::close);

setupUI();
initializeRosServicesForWidgets();
updateLayout();
}

void SecondaryWindow::closeEvent(QCloseEvent* event_)
{
    if (event_)
    {
        event_->accept();
    }
    QApplication::closeAllWindows();
}

SecondaryWindow::~SecondaryWindow()
{
    delete _ui; 
}

void SecondaryWindow::loadPredefinedStreams()
{
    for (const auto& [name, url] : Constants::CameraInfo::CAMERA_URL_MAP) {
        _predefinedStreams.push_back({QString::fromStdString(name), QString::fromStdString(url)});
    }
    
    // Create a vector of just the IPs to pass to loadPredefinedIPs
    std::vector<QString> ips;
    for (const auto& stream : _predefinedStreams) {
        // Extract IP from URL instead of using the full URL
        QString ip = extractIpFromUrl(stream.url);
        if (!ip.isEmpty()) {
            ips.push_back(ip);
        }
    }
    
    // Now pass the compatible vector type with only IPs
    _cameraSettings->loadPredefinedIPs(ips);
}

void SecondaryWindow::setupUI()
{
    connect(_ui->layoutSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &SecondaryWindow::onLayoutChange);
    
    connect(_ui->cameraSettingsButton, &QPushButton::clicked,
        this, &SecondaryWindow::showCameraSettings);

    for (int i = 0; i < static_cast<int>(_activeStreams.size()); i++) {
        connect(_activeStreams[i].widget.get(), &RtspPlayerWidget::streamStateChanged,
                this, [this, i](bool running, int) { 
                    this->onStreamStateChanged(running, static_cast<int>(i)); 
                });
                
        addStreamSelector(_activeStreams[i].widget.get(), i);
        
        for (const auto& stream : _predefinedStreams) {
            _activeStreams[i].widget->addPredefinedStream(stream.name, stream.url);
        }
    }
}

void SecondaryWindow::addStreamSelector(RtspPlayerWidget* widget, int position)
{
    QComboBox* selector = widget->findChild<QComboBox*>("streamSelector");
    if (!selector) {
        qDebug() << "Warning: Could not find streamSelector in widget" << position;
        return;
    }
    
    selector->clear();
    disconnect(selector, QOverload<int>::of(&QComboBox::currentIndexChanged), nullptr, nullptr);
    
    connect(selector, QOverload<int>::of(&QComboBox::currentIndexChanged),
            [this, position](int index) {
                if (position < 0 || position >= MAX_STREAMS) {
                    return;
                }
                
                if (index >= 0 && index < static_cast<int>(_predefinedStreams.size())) {
                    int predefinedIndex = index; 
                    
                    _activeStreams[position].predefinedStreamIndex = predefinedIndex;
                    
                    QLineEdit* urlInput = _activeStreams[position].widget->findChild<QLineEdit*>("rtspUrlInput");
                    if (urlInput) {
                        urlInput->setText(_predefinedStreams[predefinedIndex].url);
                    }
                }
            });
}

void SecondaryWindow::updateLayout()
{
    // Determine layout mode
    int layoutMode = _ui->layoutSelector->currentIndex();
    
    // Clear layouts first
    while (_ui->singleStreamStack->count() > 0) {
        QWidget* widget = _ui->singleStreamStack->widget(0);
        _ui->singleStreamStack->removeWidget(widget);
    }
    
    while (_ui->multiStreamGrid->count() > 0) {
        QLayoutItem* item = _ui->multiStreamGrid->takeAt(0);
        if (item->widget()) {
            item->widget()->setParent(nullptr);
        }
        delete item;
    }
    
    // Set grid spacing
    _ui->multiStreamGrid->setSpacing(1);
    
    // Determine number of streams for this layout
    int numStreams = 0;
    switch (layoutMode) {
        case 0: numStreams = 1; break; // Single stream
        case 1: numStreams = 4; break; // 4 streams
        case 2: numStreams = 6; break; // 6 streams
    }
    
    // Update layouts based on mode
    switch(layoutMode) {
        case 0: // Single Stream
        {
            // Add single stream to stacked widget
            for (int i = 0; i < numStreams; i++) {
                // Need heap allocation for this container as it will be owned by the stack widget
                QWidget* persistentContainer = new QWidget();
                QVBoxLayout* persistentLayout = new QVBoxLayout(persistentContainer);
                persistentLayout->setContentsMargins(0, 0, 0, 0);
                persistentLayout->addWidget(_activeStreams[i].headerLabel.get());
                persistentLayout->addWidget(_activeStreams[i].widget.get());
                
                // Add container to stacked widget
                _ui->singleStreamStack->addWidget(persistentContainer);
            }
            
            // Show the current stream
            if (_ui->singleStreamStack->count() > 0) {
                _ui->singleStreamStack->setCurrentIndex(0);
            }
            
            // Show single stream view
            _ui->layoutStack->setCurrentWidget(_ui->singleStreamView);
            break;
        }
            
        case 1: // 4 Streams
        case 2: // 6 Streams
        {
            int cols = 2;
            
            for (int i = 0; i < numStreams; i++) {
                // Add widget directly to grid layout
                int row = i / cols;
                int col = i % cols;
                _ui->multiStreamGrid->addWidget(_activeStreams[i].widget.get(), row, col);
            }
            
            _ui->layoutStack->setCurrentWidget(_ui->multiStreamView);
            break;
        }
    }
}

void SecondaryWindow::onLayoutChange(int)
{
    updateLayout();
}

void SecondaryWindow::onStreamStateChanged(bool running, int streamIndex)
{
    if (streamIndex >= 0 && streamIndex < static_cast<int>(_activeStreams.size())) {
        _activeStreams[streamIndex].isRunning = running;
    }
}

void SecondaryWindow::initializeRosServicesForWidgets()
{
    for (auto& stream : _activeStreams) {
        try {
            if (stream.widget && _node) {
                stream.widget->initializeRosServices(_node);
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(_node->get_logger(), "Failed to initialize ROS services for stream widget: %s", e.what());
        }
    }
}

void SecondaryWindow::showCameraSettings()
{
    _cameraSettings->showSettings();
}

QString SecondaryWindow::extractIpFromUrl(const QString& url)
{
    static QRegularExpression ipRegex("rtsp://(?:[^:@]+(?::[^@]+)?@)?([^:/]+)");
    QRegularExpressionMatch match = ipRegex.match(url);
    if (match.hasMatch()) {
        return match.captured(1);
    }
    return QString();
}