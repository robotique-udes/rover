#include "UI_SideBar.h"

class QSideBar : public QWidget
{
	Q_OBJECT

	public:
		QSideBar(QWidget* parent_): QWidget(parent_)
		{
			_ui.setupUi(this);

			connect(_ui.pb_dashboard, &QPushButton::clicked, this, [this]() { emit switchPage(0); });
			connect(_ui.pb_navigation, &QPushButton::clicked, this, [this]() { emit switchPage(1); });
			connect(_ui.pb_science, &QPushButton::clicked, this, [this]() { emit switchPage(2); });
		}
		~QSideBar(){};

	signals:
		void switchPage(int pageIndex);

	private:
		Ui::SideBar _ui;
};