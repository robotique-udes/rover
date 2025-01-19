#include "UI_SideBar.h"

class QDashboard : public QWidget
{
	Q_OBJECT

	public:
		QDashboard(QWidget* parent_) : QWidget(parent_)
		{
			QGridLayout* dashboardLayout = new QGridLayout(this);
			QLabel* dashboardLabel = new QLabel("Dashboard", this);
			
			dashboardLabel->setAlignment(Qt::AlignCenter);
			dashboardLayout->addWidget(dashboardLabel);

			setLayout(dashboardLayout);
		}
		~QDashboard(){};
};