#include "QController.hpp"

#include <QComboBox>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QPixmap>
#include <QSaveFile>
#include <QSignalBlocker>
#include <QStringList>
#include <utility>

namespace
{
QList<QJsonObject> parseControllerDevices(const QString &output)
{
    QList<QJsonObject> devices;
    for (const QString &line : output.split(QLatin1Char('\n')))
    {
        const QString trimmedLine = line.trimmed();
        if (trimmedLine.isEmpty() || trimmedLine.startsWith(QStringLiteral("---")) ||
            trimmedLine.startsWith(QStringLiteral("ID")))
        {
            continue;
        }

        const QChar separator = trimmedLine.contains(QLatin1Char('|')) ? QLatin1Char('|') : QLatin1Char(':');
        const QStringList fields = trimmedLine.split(separator);
        bool validId = false;
        const int deviceId = fields.isEmpty() ? -1 : fields[0].trimmed().toInt(&validId);
        if (fields.size() < 5 || !validId || deviceId < 0)
        {
            continue;
        }

        QJsonObject device;
        device.insert(QStringLiteral("id"), QString::number(deviceId));
        device.insert(QStringLiteral("sdl_guid"), fields[1].trimmed());
        device.insert(QStringLiteral("sdl_name"), fields.mid(4).join(separator).trimmed());
        devices.append(device);
    }
    return devices;
}

QString deviceSignature(const QJsonObject &device)
{
    return QStringLiteral("%1:%2:%3")
        .arg(device.value(QStringLiteral("id")).toString(),
             device.value(QStringLiteral("sdl_guid")).toString(),
             device.value(QStringLiteral("sdl_name")).toString());
}

QString controllerRegistryPath()
{
    const QString configuredPath = qEnvironmentVariable("ROVER_CONTROLLER_REGISTRY");
    if (!configuredPath.isEmpty())
    {
        return configuredPath;
    }
    return QDir::home().filePath(QStringLiteral(".config/rover/controllers.json"));
}

QJsonArray readControllerProfiles(const QString &path, bool *ok)
{
    *ok = true;
    QFile file(path);
    if (!file.exists())
    {
        return {};
    }
    if (!file.open(QIODevice::ReadOnly))
    {
        *ok = false;
        return {};
    }

    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !document.isObject() ||
        !document.object().value(QStringLiteral("controllers")).isArray())
    {
        *ok = false;
        return {};
    }
    return document.object().value(QStringLiteral("controllers")).toArray();
}

QString controllerTypeFromDevice(const QString &deviceName, const QString &guid)
{
    const QString normalizedName = deviceName.toLower();
    const QString normalizedGuid = guid.toLower();

    if (normalizedName.contains(QStringLiteral("xbox")) ||
        normalizedName.contains(QStringLiteral("x-input")) ||
        normalizedName.contains(QStringLiteral("xinput")) ||
        normalizedGuid.mid(8, 4) == QStringLiteral("5e04"))
    {
        return QStringLiteral("Xbox");
    }

    if (normalizedName.contains(QStringLiteral("logitech")) ||
        normalizedName.contains(QStringLiteral("rumblepad")) ||
        normalizedName.contains(QStringLiteral("f310")) ||
        normalizedName.contains(QStringLiteral("f510")) ||
        normalizedName.contains(QStringLiteral("f710")) ||
        normalizedGuid.mid(8, 4) == QStringLiteral("6d04"))
    {
        return QStringLiteral("Logitech");
    }

    if (normalizedName.contains(QStringLiteral("dualshock")) ||
        normalizedName.contains(QStringLiteral("sony")) ||
        normalizedGuid.mid(8, 4) == QStringLiteral("4c05") ||
        normalizedName.contains(QStringLiteral("wireless controller")))
    {
        return QStringLiteral("PS4");
    }

    return {};
}

QString registeredControllerType(const QJsonObject &device, const QJsonArray &profiles)
{
    for (const QJsonValue &value : profiles)
    {
        const QJsonObject profile = value.toObject();
        if (profile.value(QStringLiteral("sdl_guid")).toString().compare(
                device.value(QStringLiteral("sdl_guid")).toString(), Qt::CaseInsensitive) == 0 &&
            profile.value(QStringLiteral("sdl_name")).toString() ==
                device.value(QStringLiteral("sdl_name")).toString())
        {
            return profile.value(QStringLiteral("type")).toString();
        }
    }
    return {};
}

} // namespace

QController::QController(std::shared_ptr<rclcpp::Node> guiNode_, QWidget *parent_)
    : QWidget(parent_), _node(std::move(guiNode_))
{
    _ui.setupUi(this);
    _ui.label->setAlignment(Qt::AlignCenter);
    _ui.controllerTypeComboBox->hide();
    _ui.controllerTypeLabel->hide();
    _ui.registrationStatusLabel->hide();
    _ui.addControllerButton->setEnabled(false);
    _ui.label->clear();
    _ui.label->setText(QStringLiteral("Recherche d’une manette…"));

    connect(_ui.addControllerButton,
            &QPushButton::clicked,
            this,
            &QController::toggleControllerEnrollment);

    connect(&_enumerationProcess,
            qOverload<int, QProcess::ExitStatus>(&QProcess::finished),
            this,
            &QController::handleEnumerationFinished);
    connect(&_enumerationProcess,
            &QProcess::errorOccurred,
            this,
            [this](QProcess::ProcessError)
            {
                if (!_enumerationErrorLogged)
                {
                    RCLCPP_ERROR(_node->get_logger(),
                                 "Could not start joy_enumerate_devices for controller detection");
                    _enumerationErrorLogged = true;
                }
            });

    _enumerationTimer.setInterval(1000);
    connect(&_enumerationTimer, &QTimer::timeout, this, &QController::enumerateControllers);
    _manualFallbackTimer.setSingleShot(true);
    _manualFallbackTimer.setInterval(5000);
    connect(&_manualFallbackTimer, &QTimer::timeout, this, &QController::offerManualSelection);

    connect(_ui.controllerTypeComboBox,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this,
            [this](int index)
            {
                if (index > 0)
                {
                    const QString selectedType = _ui.controllerTypeComboBox->itemText(index);
                    if (_enrollmentActive)
                    {
                        _enrollmentType = selectedType;
                        savePendingControllerProfile();
                    }
                    else if (_manualSelectionOffered)
                    {
                        _manualTypeChosen = true;
                        _ui.label->setText(QString());
                        updateControllerImage(selectedType);
                    }
                }
            });

    _manualFallbackTimer.start();
    enumerateControllers();
    _enumerationTimer.start();
}

void QController::enumerateControllers()
{
    if (_enumerationProcess.state() == QProcess::NotRunning)
    {
        _enumerationProcess.start(
            QStringLiteral("ros2"),
            {QStringLiteral("run"), QStringLiteral("joy"), QStringLiteral("joy_enumerate_devices")});
    }
}

void QController::handleEnumerationFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    if (exitStatus != QProcess::NormalExit || exitCode != 0)
    {
        if (!_enumerationErrorLogged)
        {
            RCLCPP_ERROR(_node->get_logger(),
                         "joy_enumerate_devices failed with exit code %d",
                         exitCode);
            _enumerationErrorLogged = true;
        }
        return;
    }

    _enumerationErrorLogged = false;
    const QList<QJsonObject> devices =
        parseControllerDevices(QString::fromUtf8(_enumerationProcess.readAllStandardOutput()));
    bool profilesOk = false;
    const QJsonArray profiles = readControllerProfiles(controllerRegistryPath(), &profilesOk);
    if (!profilesOk)
    {
        if (!_profileReadErrorLogged)
        {
            RCLCPP_ERROR(_node->get_logger(), "Controller profile registry is invalid or unreadable");
            _profileReadErrorLogged = true;
        }
        _ui.registrationStatusLabel->setText(
            QStringLiteral("Impossible de lire les profils de manettes enregistrés."));
        _ui.registrationStatusLabel->show();
        return;
    }
    _profileReadErrorLogged = false;

    QSet<QString> currentSignatures;
    QList<QJsonObject> newlyConnectedDevices;
    QString recognizedType;
    for (const QJsonObject &device : devices)
    {
        const QString signature = deviceSignature(device);
        currentSignatures.insert(signature);
        if (!_knownDeviceSignatures.contains(signature))
        {
            newlyConnectedDevices.append(device);
        }

        QString detectedType = registeredControllerType(device, profiles);
        if (detectedType.isEmpty())
        {
            detectedType = controllerTypeFromDevice(
                device.value(QStringLiteral("sdl_name")).toString(),
                device.value(QStringLiteral("sdl_guid")).toString());
        }
        if (recognizedType.isEmpty() && !detectedType.isEmpty())
        {
            recognizedType = detectedType;
        }
    }

    _knownDeviceSignatures = currentSignatures;
    _hasDeviceSnapshot = true;
    _ui.addControllerButton->setEnabled(true);

    if (_enrollmentActive && !newlyConnectedDevices.isEmpty())
    {
        if (newlyConnectedDevices.size() == 1)
        {
            _pendingNewDevices = newlyConnectedDevices;
            savePendingControllerProfile();
        }
        else
        {
            _pendingNewDevices.clear();
            _ui.registrationStatusLabel->setText(
                QStringLiteral("Plusieurs nouvelles manettes détectées. Débranche-les, puis reconnecte-en une seule."));
            _ui.registrationStatusLabel->show();
            RCLCPP_WARN(_node->get_logger(),
                        "Multiple newly connected controllers found during profile enrollment");
        }
    }

    if (recognizedType.isEmpty() || _manualTypeChosen)
    {
        return;
    }

    _manualFallbackTimer.stop();
    _ui.controllerTypeLabel->hide();
    _ui.controllerTypeComboBox->hide();
    {
        const QSignalBlocker blocker(_ui.controllerTypeComboBox);
        const int index = _ui.controllerTypeComboBox->findText(recognizedType);
        if (index >= 0)
        {
            _ui.controllerTypeComboBox->setCurrentIndex(index);
        }
    }
    _ui.label->setText(QStringLiteral("Manette %1 détectée").arg(recognizedType));
    updateControllerImage(recognizedType);
}

void QController::offerManualSelection()
{
    _manualSelectionOffered = true;
    _ui.controllerTypeLabel->show();
    _ui.controllerTypeComboBox->show();
    if (!_enrollmentActive)
    {
        _ui.registrationStatusLabel->hide();
        _ui.label->setText(QStringLiteral("Aucune manette reconnue après 5 secondes. Choisis le type manuellement."));
    }
}

void QController::toggleControllerEnrollment()
{
    if (_enrollmentActive)
    {
        _enrollmentActive = false;
        _pendingNewDevices.clear();
        _enrollmentType.clear();
        _ui.addControllerButton->setText(QStringLiteral("Ajouter une manette"));
        _ui.registrationStatusLabel->setText(QStringLiteral("Ajout annulé."));
        _ui.registrationStatusLabel->show();
        return;
    }

    if (!_hasDeviceSnapshot)
    {
        _ui.registrationStatusLabel->setText(
            QStringLiteral("La détection SDL n’est pas encore prête. Réessaie dans un instant."));
        _ui.registrationStatusLabel->show();
        return;
    }

    _enrollmentActive = true;
    _enumerationTimer.start();
    _pendingNewDevices.clear();
    _enrollmentType.clear();
    _ui.controllerTypeLabel->show();
    _ui.controllerTypeComboBox->show();
    _ui.controllerTypeComboBox->setCurrentIndex(0);
    _ui.addControllerButton->setText(QStringLiteral("Annuler l’ajout"));
    _ui.registrationStatusLabel->setText(
        QStringLiteral("Choisis le type, puis connecte une seule nouvelle manette (USB ou Bluetooth)."));
    _ui.registrationStatusLabel->show();
}

void QController::savePendingControllerProfile()
{
    if (!_enrollmentActive || _enrollmentType.isEmpty() || _pendingNewDevices.size() != 1)
    {
        return;
    }

    const QString path = controllerRegistryPath();
    const QFileInfo registryInfo(path);
    if (!QDir().mkpath(registryInfo.absolutePath()))
    {
        RCLCPP_ERROR(_node->get_logger(), "Could not create controller profile directory");
        _ui.registrationStatusLabel->setText(
            QStringLiteral("Impossible de créer le dossier du registre de manettes."));
        _ui.registrationStatusLabel->show();
        return;
    }

    bool profilesOk = false;
    QJsonArray profiles = readControllerProfiles(path, &profilesOk);
    if (!profilesOk)
    {
        RCLCPP_ERROR(_node->get_logger(), "Controller profile registry is invalid or unreadable");
        _ui.registrationStatusLabel->setText(
            QStringLiteral("Le registre de manettes est invalide ou illisible."));
        _ui.registrationStatusLabel->show();
        return;
    }

    const QJsonObject device = _pendingNewDevices.front();
    QJsonObject profile;
    profile.insert(QStringLiteral("type"), _enrollmentType);
    profile.insert(QStringLiteral("sdl_guid"), device.value(QStringLiteral("sdl_guid")));
    profile.insert(QStringLiteral("sdl_name"), device.value(QStringLiteral("sdl_name")));

    bool updated = false;
    for (qsizetype i = 0; i < profiles.size(); ++i)
    {
        const QJsonObject existing = profiles[i].toObject();
        if (existing.value(QStringLiteral("sdl_guid")).toString().compare(
                profile.value(QStringLiteral("sdl_guid")).toString(), Qt::CaseInsensitive) == 0 &&
            existing.value(QStringLiteral("sdl_name")).toString() ==
                profile.value(QStringLiteral("sdl_name")).toString())
        {
            profiles.replace(i, profile);
            updated = true;
            break;
        }
    }
    if (!updated)
    {
        profiles.append(profile);
    }

    QJsonObject registry;
    registry.insert(QStringLiteral("version"), 1);
    registry.insert(QStringLiteral("controllers"), profiles);
    QSaveFile output(path);
    if (!output.open(QIODevice::WriteOnly) ||
        output.write(QJsonDocument(registry).toJson(QJsonDocument::Indented)) < 0 ||
        !output.commit())
    {
        RCLCPP_ERROR(_node->get_logger(), "Could not save controller profile registry");
        _ui.registrationStatusLabel->setText(
            QStringLiteral("Impossible d’enregistrer le profil de cette manette."));
        _ui.registrationStatusLabel->show();
        return;
    }

    _enrollmentActive = false;
    _pendingNewDevices.clear();
    _ui.addControllerButton->setText(QStringLiteral("Ajouter une manette"));
    _ui.registrationStatusLabel->setText(
        QStringLiteral("Manette %1 enregistrée. Redémarre le lancement ROS pour appliquer son mapping.")
            .arg(_enrollmentType));
    _ui.registrationStatusLabel->show();
    _ui.label->setText(QStringLiteral("Manette %1 enregistrée").arg(_enrollmentType));
    updateControllerImage(_enrollmentType);
    _enrollmentType.clear();
    RCLCPP_INFO(_node->get_logger(), "Saved controller profile in %s", path.toStdString().c_str());
}

void QController::updateControllerImage(const QString &controllerType)
{
    QString iconPath;
    if (controllerType == QStringLiteral("DS4") || controllerType == QStringLiteral("PS4"))
    {
        iconPath = QStringLiteral(":/icons/PS4_Controller.svg");
    }
    else if (controllerType == QStringLiteral("Logitech") ||
             controllerType == QStringLiteral("Logitech Generic"))
    {
        iconPath = QStringLiteral(":/icons/Logitech_Controller.svg");
    }
    else if (controllerType == QStringLiteral("Xbox") || controllerType == QStringLiteral("XBox"))
    {
        iconPath = QStringLiteral(":/icons/XBox_Controller.svg");
    }
    else
    {
        _ui.label->clear();
        _ui.label->setText(QStringLiteral("Type de manette inconnu : %1").arg(controllerType));
        RCLCPP_WARN(rclcpp::get_logger("QController"),
                    "Unknown controller type received: %s",
                    controllerType.toStdString().c_str());
        return;
    }

    const QPixmap controllerPixmap(iconPath);
    if (controllerPixmap.isNull())
    {
        _ui.label->clear();
        _ui.label->setText(QStringLiteral("Impossible de charger l’image de la manette"));
        RCLCPP_ERROR(rclcpp::get_logger("QController"),
                     "Failed to load controller icon: %s",
                     iconPath.toStdString().c_str());
        return;
    }

    _ui.label->setText(QString());
    _ui.label->setPixmap(controllerPixmap.scaled(
        _ui.label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}
