#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

class QNavigation : public QWidget
{
	Q_OBJECT

	public:
		QNavigation(QWidget* parent_) : QWidget(parent_)
		{
			QGridLayout* navigationLayout = new QGridLayout(this);
			QLabel* navigationLabel = new QLabel("Navigation", this);
			
			navigationLabel->setAlignment(Qt::AlignCenter);
			navigationLayout->addWidget(navigationLabel);

			setLayout(navigationLayout);
		}
		~QNavigation(){};
};