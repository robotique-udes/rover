#include "rclcpp/rclcpp.hpp"
#include <QtWidgets/QWidget>
#include <QtWidgets/QGridLayout>

#include "QExample.hpp"

class QDashboard : public QWidget
{
	Q_OBJECT

	public:
		QDashboard(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_)
		 : QWidget(parent_), _node(guiNode_),_exampleWidget(guiNode_, parent_)
		{
			QGridLayout* dashboardLayout = new QGridLayout(this);
			
			// Add your dashboard widget here
			dashboardLayout->addWidget(&_exampleWidget);

			setLayout(dashboardLayout);
		}
		~QDashboard(){};

	private:
		std::shared_ptr<rclcpp::Node> _node;
		QExample _exampleWidget;
};