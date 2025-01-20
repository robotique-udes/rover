#include "rclcpp/rclcpp.hpp"

#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

class QNavigation : public QWidget
{
	Q_OBJECT

	public:
		QNavigation(std::shared_ptr<rclcpp::Node> _guiNode, QWidget* parent_)
		 : QWidget(parent_), _node(_guiNode)
		{
			QGridLayout* navigationLayout = new QGridLayout(this);
			QLabel* navigationLabel = new QLabel("Navigation", this);
			
			navigationLabel->setAlignment(Qt::AlignCenter);
			navigationLayout->addWidget(navigationLabel);

			setLayout(navigationLayout);
		}
		~QNavigation(){};
	private:
		std::shared_ptr<rclcpp::Node> _node;
};