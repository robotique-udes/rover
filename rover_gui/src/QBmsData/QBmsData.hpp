#ifndef QBMSDATA_HPP
#define QBMSDATA_HPP

#include <QtWidgets/QGridLayout>


class QBmsData : public QWidget
{
    Q_OBJECT

    public:
        QArbitration(std::shared_ptr<rclcpp::Node> guiNode_, QWidget* parent_);


    private:

};

#endif