#include "QUtilityBarTop.hpp"
#include "rover_lib2/helpers/constants.hpp"
#include <QIcon>
#include <QFile>
#include <QStyle>
#include <qdebug.h>
#include <qtimezone.h>

#include <cmath>

namespace
{
    QString toDMS(double deg, bool isLat)
    {
        char dir;

        if (isLat)
            dir = (deg >= 0) ? 'N' : 'S';
        else
            dir = (deg >= 0) ? 'E' : 'W';

        deg = std::abs(deg);

        int d = static_cast<int>(deg);
        double minFloat = (deg - d) * 60.0;
        int m = static_cast<int>(minFloat);
        double s = (minFloat - m) * 60.0;

        return QString("%1° %2' %3\" %4").arg(d).arg(m).arg(s, 0, 'f', 2).arg(dir);
    }
}  // namespace

QUtilityBarTop::QUtilityBarTop(std::shared_ptr<rclcpp::Node> node_, QWidget* parent_):
    QWidget(parent_),
    _node(node_),
    _batteryTimeout(rclcpp::Duration::from_seconds(WATCH_DOG_TIMEOUT)),
    _GNSSTimeout(rclcpp::Duration::from_seconds(WATCH_DOG_TIMEOUT)),
    _antennaTimeout(rclcpp::Duration::from_seconds(WATCH_DOG_TIMEOUT))
{
    _ui.setupUi(this);

    _lastBatteryTimeMsg = _node->now();
    _lastAntennaTimeMsg = _node->now();
    _lastGNSSTimeMsg = _node->now();

    this->setupUI();
    this->initBatterySubscriber();
    this->initAntennaStatus();
    this->initGNSS();
    this->initTimerDisplay();

    connect(this, &QUtilityBarTop::updateBatteryUI, this, &QUtilityBarTop::onUpdateBatteryUI);
    connect(this, &QUtilityBarTop::updateAntennaUI, this, &QUtilityBarTop::onUpdateAntennaUI);
    connect(this, &QUtilityBarTop::updateGNSS, this, &QUtilityBarTop::onUpdateGNSS);
    connect(this, &QUtilityBarTop::updateTimer, this, &QUtilityBarTop::onUpdateTimer);

    connect(this, &QUtilityBarTop::batteryPubCount, this, &QUtilityBarTop::onBatteryPubCount);
    connect(this, &QUtilityBarTop::antennaStatusPubCount, this, &QUtilityBarTop::onAntennaStatusPubCount);
    connect(this, &QUtilityBarTop::GNSSPubCount, this, &QUtilityBarTop::onGNSSPubCount);

    connect(this, &QUtilityBarTop::batteryTimeout, this, &QUtilityBarTop::onBatteryTimeout);
    connect(this, &QUtilityBarTop::antennaStatusTimeout, this, &QUtilityBarTop::onAntennaStatusTimeout);
    connect(this, &QUtilityBarTop::GNSSTimeout, this, &QUtilityBarTop::onGNSSTimeout);

    connect(this, &QUtilityBarTop::timerDisplay, this, &QUtilityBarTop::onTimerDisplay);
    connect(_ui.dmsToggle, &QSlider::valueChanged, this, &QUtilityBarTop::dmsChanged);
}

void QUtilityBarTop::setupUI(void)
{
    _timeZone = QTimeZone("America/Montreal");

    _ui.batteryLabel->setText("-- %");
    _ui.signalQualityLabel->setText("RSSI: ---");
    _ui.upSpeedLabel->setText("--.- Mb/s");
    _ui.downSpeedLabel->setText("--.- Mb/s");
    _ui.satellitesNbrLabel->setText("Sat: --");
    _ui.GNSSFixLabel->setText("Fix: --.-------");
    _ui.HeadingLabel->setText("---.--");

    _ui.upSpeedButton->setIcon(QIcon::fromTheme("go-up"));
    _ui.downSpeedButton->setIcon(QIcon::fromTheme("go-down"));

    this->setStyleSheet(R"(
    QPushButton {
        background-color: transparent;
        border: none;
    }
    )");
}

void QUtilityBarTop::initBatterySubscriber(void)
{
    QIcon icon(":/icons/BatteryError.svg");
    _ui.batteryIcon->setIcon(icon);

    if (_node)
    {
        _sub_battery = _node->create_subscription<rover_msgs::msg::Battery>(TOPIC_BATTERY,
                                                                            QOS_DEFAULT,
                                                                            [this](rover_msgs::msg::Battery msg_)
                                                                            {
                                                                                emit this->updateBatteryUI(msg_.state_of_charge);
                                                                            });

        _timer_batteryPub = _node->create_wall_timer(std::chrono::milliseconds(DELAY_CHECK_BATTERY_PUB_COUNT_MS),
                                                     [this](void)
                                                     {
                                                         emit this->batteryPubCount();
                                                     });

        _watchdog_battery = _node->create_wall_timer(std::chrono::milliseconds(WATCH_DOG_DELAY_MS),
                                                     [this](void)
                                                     {
                                                         emit this->batteryTimeout();
                                                     });
    }
    else
    {
        assert(false && "Error: GUI node is null");
    }
}

void QUtilityBarTop::initAntennaStatus(void)
{
    QIcon icon(":/icons/RSSIError.svg");
    _ui.RSSILabel->setIcon(icon);

    if (_node)
    {
        _sub_antennaStatus = _node->create_subscription<rover_msgs::msg::AntennaStatus>(
            TOPIC_ANTENNA_STATUS,
            QOS_DEFAULT,
            [this](rover_msgs::msg::AntennaStatus msg_)
            {
                emit this->updateAntennaUI(msg_.connected, msg_.rssi, msg_.txrate, msg_.rxrate);
            });

        _timer_RSSIPub = _node->create_wall_timer(std::chrono::milliseconds(DELAY_CHECK_RSSI_PUB_COUNT_MS),
                                                  [this](void)
                                                  {
                                                      emit this->antennaStatusPubCount();
                                                  });

        _watchdog_antenna = _node->create_wall_timer(std::chrono::milliseconds(WATCH_DOG_DELAY_MS),
                                                     [this](void)
                                                     {
                                                         emit this->antennaStatusTimeout();
                                                     });
    }
    else
    {
        assert(false && "Error: GUI node is null");
    }
}

void QUtilityBarTop::initGNSS(void)
{
    if (_node)
    {
        _sub_GNSS = _node->create_subscription<rover_msgs::msg::Gps>(
            TOPIC_GNSS,
            QOS_DEFAULT,
            [this](rover_msgs::msg::Gps msg_)
            {
                emit this->updateGNSS(msg_.fix_quality, msg_.heading, msg_.satellite, msg_.longitude, msg_.latitude);
            });

        _timer_GNSSPub = _node->create_wall_timer(std::chrono::milliseconds(DELAY_CHECK_GNSS_PUB_COUNT_MS),
                                                  [this](void)
                                                  {
                                                      emit this->GNSSPubCount();
                                                  });

        _watchdog_GNSS = _node->create_wall_timer(std::chrono::milliseconds(WATCH_DOG_DELAY_MS),
                                                  [this](void)
                                                  {
                                                      emit this->GNSSTimeout();
                                                  });
    }
    else
    {
        assert(false && "Error: GUI node is null");
    }
}

void QUtilityBarTop::initTimerDisplay(void)
{
    this->readTimersFromFile(FILE_PATH, _timersList);
    _timer_updateTimer = _node->create_wall_timer(std::chrono::milliseconds(DELAY_UPDATE_TIMER_MS),
                                                  [this](void)
                                                  {
                                                      emit this->timerDisplay();
                                                  });
}

void QUtilityBarTop::onTimerDisplay(void)
{
    int secondsBeforeTimeout = -1;

    for (const auto& timers : _timersList)
    {
        QDateTime givenTimeUTC = timers.toUTC();
        QDateTime nowUtc = QDateTime::currentDateTimeUtc();

        int secondsDiff = nowUtc.secsTo(givenTimeUTC);

        if ((secondsDiff < secondsBeforeTimeout && secondsDiff > 0) || (secondsBeforeTimeout == -1 && secondsDiff > 0))
        {
            secondsBeforeTimeout = secondsDiff;
        }
    }

    emit this->updateTimer(secondsBeforeTimeout);
}

void QUtilityBarTop::onUpdateBatteryUI(float _percent)
{
    _ui.batteryLabel->setText(QString::number(static_cast<float>(std::round(_percent))) + " %");
    QIcon icon;

    if (_percent >= 85)
    {
        icon = QIcon(":/icons/BatteryIcon100.svg");
    }
    else if (_percent >= 55)
    {
        icon = QIcon(":/icons/BatteryIcon75.svg");
    }
    else if (_percent >= 40)
    {
        icon = QIcon(":/icons/BatteryIcon50.svg");
    }
    else if (_percent >= 20)
    {
        icon = QIcon(":/icons/BatteryIcon25.svg");
    }
    else
    {
        icon = QIcon(":/icons/BatteryIcon0.svg");
    }

    _ui.batteryIcon->setIcon(icon);
}

void QUtilityBarTop::onUpdateAntennaUI(bool connected_, float rssi_, float upSpeed_, float downSpeed_)
{
    QIcon icon;
    if (connected_)
    {
        _ui.signalQualityLabel->setText("RSSI: " + QString::number(static_cast<int>(rssi_)));
        _ui.upSpeedLabel->setText(QString::number(static_cast<float>(upSpeed_ / 1000000.0f), 'f', 1) + " Mb/s");
        _ui.downSpeedLabel->setText(QString::number(static_cast<float>(downSpeed_ / 1000000.0f), 'f', 1) + " Mb/s");

        if (rssi_ <= -85)
        {
            icon = QIcon(":/icons/RSSI_one.png");
        }
        else if (rssi_ > -85 && rssi_ <= -75)
        {
            icon = QIcon(":/icons/RSSI_two.png");
        }
        else if (rssi_ > -75 && rssi_ <= -65)
        {
            icon = QIcon(":/icons/RSSI_three.png");
        }
        else if (rssi_ == 0)
        {
            icon = QIcon(":/icons/ErrorRSSI.png");
        }
        else
        {
            icon = QIcon(":/icons/RSSI_four.png");
        }
    }
    else
    {
        _ui.signalQualityLabel->setText("RSSI: ---");
        _ui.upSpeedLabel->setText("--.- Mb/s");
        icon = QIcon(":/icons/ErrorRSSI.png");
    }

    _ui.RSSILabel->setIcon(icon);
}

void QUtilityBarTop::onUpdateGNSS(uint8_t fix_, float heading_, uint8_t satNbr_, float long_, float lat_)
{
    _ui.HeadingLabel->setText(QString::number((heading_), 'f', 2) + " deg   ");
    _ui.satellitesNbrLabel->setText(QString::number(static_cast<int>(satNbr_)) + "   ");

    if (_ui.dmsToggle->value() == 0)
    {
        _ui.latitudeLabel->setText("Lat: " + QString::number(static_cast<float>(lat_), 'f', 6));
        _ui.longitudeLabel->setText("Long: " + QString::number(static_cast<float>(long_), 'f', 6));
    }
    else
    {
        _ui.latitudeLabel->setText("Lat: " + toDMS(lat_, true));
        _ui.longitudeLabel->setText("Long: " + toDMS(long_, false));
    }

    _ui.satellliteIcon_pb->setIcon(QIcon(":/icons/GNSSIcon.svg"));
    _ui.headingIcon_pb->setIcon(QIcon(":/icons/HeadingIcon.svg"));

    _ui.longitudeLabel->setStyleSheet("QLabel { color : white; }");
    _ui.latitudeLabel->setStyleSheet("QLabel { color : white; }");

    std::string fixQuality;

    switch (fix_)
    {
        case rover_msgs::msg::Gps::FIX_QUALITY_NO_FIX:
            fixQuality = "No Fix";
            break;
        case rover_msgs::msg::Gps::FIX_QUALITY_GPS:
            fixQuality = "GPS";
            break;
        case rover_msgs::msg::Gps::FIX_QUALITY_GNSS:
            fixQuality = "GNSS";
            break;
        case rover_msgs::msg::Gps::FIX_QUALITY_RTK:
            fixQuality = "RTK";
            break;
        default:
            fixQuality = "No Fix";
            break;
    }

    _ui.GNSSFixLabel->setText("Fix Quality: " + QString::fromStdString(fixQuality));
}

void QUtilityBarTop::onUpdateTimer(int secondsBeforeTimeOut_)
{
    if (secondsBeforeTimeOut_ >= 0)
    {
        int hours = secondsBeforeTimeOut_ / 3600;
        int minutes = (secondsBeforeTimeOut_ % 3600) / 60;
        int seconds = secondsBeforeTimeOut_ % 60;

        QString timeString
            = QString("%1:%2:%3").arg(hours, 2, 10, QChar('0')).arg(minutes, 2, 10, QChar('0')).arg(seconds, 2, 10, QChar('0'));

        QDateTime now = QDateTime::currentDateTimeUtc().toTimeZone(_timeZone);
        _ui.timerLabel->setText(timeString);

        if (secondsBeforeTimeOut_ < 300)
        {
            _ui.timerLabel->setStyleSheet("QLabel { color : red; }");
        }
        else if (secondsBeforeTimeOut_ < 600)
        {
            _ui.timerLabel->setStyleSheet("QLabel { color : orange; }");
        }
        else
        {
            _ui.timerLabel->setStyleSheet("QLabel { color : white; }");
        }
    }
    else
    {
        _ui.timerLabel->setText("OVER");
        _ui.timerLabel->setStyleSheet("QLabel { color : red; }");
    }
}

void QUtilityBarTop::onBatteryPubCount()
{
    _lastBatteryTimeMsg = _node->now();
    size_t count = _node->count_publishers(TOPIC_BATTERY);
    if (!count)
    {
        QIcon icon(":/icons/BatteryError.svg");
        _ui.batteryIcon->setIcon(icon);
    }
}

void QUtilityBarTop::onAntennaStatusPubCount()
{
    _lastAntennaTimeMsg = _node->now();
    size_t count = _node->count_publishers(TOPIC_ANTENNA_STATUS);
    if (!count)
    {
        QIcon icon(":/icons/ErrorRSSI.png");
        _ui.RSSILabel->setIcon(icon);
    }
}

void QUtilityBarTop::onGNSSPubCount()
{
    _lastGNSSTimeMsg = _node->now();
    size_t count = _node->count_publishers(TOPIC_GNSS);
    if (!count)
    {
        QIcon iconSat(":/icons/GNSSError.svg");
        QIcon iconHeading(":/icons/HeadingError.svg");
        _ui.satellliteIcon_pb->setIcon(iconSat);
        _ui.headingIcon_pb->setIcon(iconHeading);
        _ui.longitudeLabel->setStyleSheet("QLabel { color : orange; }");
        _ui.latitudeLabel->setStyleSheet("QLabel { color : orange; }");
    }
}

void QUtilityBarTop::onBatteryTimeout()
{
    if ((_node->now() - _lastBatteryTimeMsg) > _batteryTimeout)
    {
        QIcon icon(":/icons/BatteryError.svg");
        _ui.batteryIcon->setIcon(icon);
    }
}
void QUtilityBarTop::onAntennaStatusTimeout()
{
    if ((_node->now() - _lastAntennaTimeMsg) > _antennaTimeout)
    {
        QIcon icon(":/icons/ErrorRSSI.png");
        _ui.RSSILabel->setIcon(icon);
    }
}
void QUtilityBarTop::onGNSSTimeout()
{
    if ((_node->now() - _lastGNSSTimeMsg) > _GNSSTimeout)
    {
        QIcon iconSat(":/icons/GNSSError.svg");
        QIcon iconHeading(":/icons/Heading");
        _ui.satellliteIcon_pb->setIcon(iconSat);
        _ui.headingIcon_pb->setIcon(iconHeading);
        _ui.longitudeLabel->setStyleSheet("QLabel { color : orange; }");
        _ui.latitudeLabel->setStyleSheet("QLabel { color : orange; }");
    }
}

void QUtilityBarTop::readTimersFromFile(const char* filename_, std::vector<QDateTime>& timersList_)
{
    QFile file(QString(":/") + filename_);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        RCLCPP_ERROR(rclcpp::get_logger("GUI"), "Unable to open file containing timers for top bar");
        return;
    }

    QTextStream in(&file);

    QString line = in.readLine();

    while (!in.atEnd())
    {
        QString line = in.readLine();
        QStringList parts = line.split(';');

        if (parts.size() == 7)
        {
            int year = parts[0].toInt();
            int month = parts[1].toInt();
            int day = parts[2].toInt();
            int hour = parts[3].toInt();
            int minute = parts[4].toInt();
            int second = parts[5].toInt();
            QString tz = parts[6].trimmed();

            QTimeZone timezone;

            if (tz == "QC")
            {
                timezone = QTimeZone("America/Montreal");
            }
            else
            {
                timezone = QTimeZone("America/Edmonton");
            }

            QDateTime dateTime = QDateTime(QDate(year, month, day), QTime(hour, minute, second), timezone);
            timersList_.push_back(dateTime);
        }
    }
}
