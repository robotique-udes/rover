#ifndef QCONTROLLER_HPP
#define QCONTROLLER_HPP

#include "ui_Controller.h"

#include <QProcess>
#include <QSet>
#include <QTimer>
#include <QWidget>

#include <QJsonObject>
#include <rclcpp/rclcpp.hpp>

class QController : public QWidget {
  Q_OBJECT

public:
  explicit QController(std::shared_ptr<rclcpp::Node> guiNode, QWidget *parent = nullptr);

private:
  void enumerateControllers();
  void handleEnumerationFinished(int exitCode, QProcess::ExitStatus exitStatus);
  void offerManualSelection();
  void toggleControllerEnrollment();
  void savePendingControllerProfile();
  void updateControllerImage(const QString &controllerType);

  Ui::Controller _ui;
  rclcpp::Node::SharedPtr _node;
  QProcess _enumerationProcess;
  QTimer _enumerationTimer;
  QTimer _manualFallbackTimer;
  QSet<QString> _knownDeviceSignatures;
  QList<QJsonObject> _pendingNewDevices;
  QString _enrollmentType;
  bool _enumerationErrorLogged = false;
  bool _profileReadErrorLogged = false;
  bool _manualSelectionOffered = false;
  bool _manualTypeChosen = false;
  bool _enrollmentActive = false;
  bool _hasDeviceSnapshot = false;
};

#endif // QCONTROLLER_HPP
