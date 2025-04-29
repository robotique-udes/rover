#ifndef QTOASTNOTIFICATION
#define QTOASTNOTIFICATION

// ROS
#include "rclcpp/rclcpp.hpp"
// QT

#include "UI_ToastNotification.h"
#include "UI_OpenNotifications.h"
#include "UI_NotificationHistory.h"
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QWidget>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QTimer>
#include <QApplication>
#include <QGraphicsDropShadowEffect>
#include <QStyle>
#include <deque>

namespace QHelper
{


  class QToastNotification : public QWidget
  {
      Q_OBJECT

      static constexpr uint64_t NOTIF_DURATION_MS = 5'000UL;
      static constexpr size_t HISTORY_MAX_SIZE = 10U;

    public:
      enum class eNotifType : uint8_t
      {
          ERROR = 0u,
          WARNING = 1u,
          INFO = 2u
      };

        struct sNotificationInfo    
      {
        #warning add time stamp
        //long long timeStamp;
        std::string title;
        std::string description;
        QToastNotification::eNotifType criticityLevel;
      };

      QRect targetScreenRect;
      std::deque<sNotificationInfo> _history; 


      static QToastNotification& getInstance();

      void notify(const std::string& title, const std::string& description_, eNotifType type_, int durationMs_ = NOTIF_DURATION_MS);

    private:
      QToastNotification();
      ~QToastNotification() = default;


      void setupUI();
      void setupAnimations();
      void setupScreenRect();        const int margin = 1;

      void hideNotification();

      void saveNotifInfo(const sNotificationInfo& info_);

      QToastNotification(const QToastNotification&) = delete;
      QToastNotification& operator=(const QToastNotification&) = delete;

      // Q_DISABLE_COPY(QSingletonToast)

      Ui::ToastNotification _ui;
      QGraphicsDropShadowEffect _shadow;
      QPropertyAnimation _fadeInAnim;
      QPropertyAnimation _fadeOutAnim;
      QPropertyAnimation _slideInAnim;
      QPropertyAnimation _slideOutAnim;
      QPropertyAnimation _progressBarAnim;

      QParallelAnimationGroup _animationGroup;

      QTimer _closeTimer;
  };

  class QNotificationShowHistory : public QWidget
  {
      Q_OBJECT

      static constexpr size_t MARGIN = 1;
    public:

      QNotificationShowHistory();
      ~QNotificationShowHistory();

      void showHistory();


    private:

      Ui::historyWidget _ui_mainWidget;
      //Ui::historySubWidget _ui_subWidget;

      QRect _targetScreenRect;
      QWidget scrollAreaContainer;
      QVBoxLayout scrollAreaLayout;
  };

  class QNotificationHistoryData: public QWidget
  {
    Q_OBJECT

    public:

      QNotificationHistoryData(QWidget* parent_, std::string title_, std::string description_, QToastNotification::eNotifType type_);
      ~QNotificationHistoryData();

    private:
      Ui::historySubWidget _ui_subWidget;
  };


}  // nameenelverspace QHelper

#endif