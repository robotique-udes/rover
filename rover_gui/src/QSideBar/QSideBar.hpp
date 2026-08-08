#ifndef QSIDE_BAR_HPP
#define QSIDE_BAR_HPP

#include "UI_SideBar.h"

class QSideBar : public QWidget
{
    Q_OBJECT

  public:
    enum class eTabIndex : int
    {
        DASHBOARD = 0,
        NAVIGATION,
        FILE_TRANSFER,
        DEVICE_STATUS,
        BMS_DATA
        SCIENCE,
    };

    explicit QSideBar(QWidget* parent_);

  signals:
    void switchPage(eTabIndex pageIndex);

  private:
    Ui::SideBar _ui;
};

#endif  // QSIDE_BAR_HPP
