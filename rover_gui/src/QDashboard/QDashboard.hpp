#include "rclcpp/rclcpp.hpp"
#include <QtWidgets/QWidget>
#include <QtWidgets/QGridLayout>

#include "QExample.hpp"

class QDashboard : public QWidget
{
	Q_OBJECT

	public:
		QDashboard(std::shared_ptr<rclcpp::Node> _guiNode, QWidget* parent_)
		 : QWidget(parent_), _node(_guiNode),_exampleWidget(parent_)
		{
			QGridLayout* dashboardLayout = new QGridLayout(this);
			dashboardLayout->addWidget(&_exampleWidget);

			setLayout(dashboardLayout);
		}
		~QDashboard(){};

	private:
		std::shared_ptr<rclcpp::Node> _node;
		QExample _exampleWidget;
};