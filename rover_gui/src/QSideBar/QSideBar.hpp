#include "UI_SideBar.h"

class QSideBar : public QWidget
{
public:
	QSideBar(QWidget* parent_): QWidget(parent_)
	{
		_ui.setupUi(this);
	}
	~QSideBar(){};

private:
	Ui::SideBar _ui;
};