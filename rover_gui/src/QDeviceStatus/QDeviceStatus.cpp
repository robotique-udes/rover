#include "QDeviceStatus.hpp"

QDeviceStatus::QDeviceStatus(QWidget* parent_) : QWidget(parent_)
{
	_ui.setupUi(this);


    // Set the QSizePolicy to ensure aspect ratio resizing
    QSizePolicy sp = this->sizePolicy();
    sp.setHorizontalPolicy(QSizePolicy::Preferred);
    sp.setVerticalPolicy(QSizePolicy::Preferred);
    sp.setHeightForWidth(true);  // Enable height for width
    this->setSizePolicy(sp);
}

/*

int QDeviceStatus::heightForWidth(int width_) const
{
    // Load the image and get the aspect ratio
    QPixmap pixmap(":/images/rover.png");  // Path to your image in resources
    int originalWidth = pixmap.width();
    int originalHeight = pixmap.height();

    // Calculate height based on width, keeping the same aspect ratio
    int height = width_ * originalHeight / originalWidth;
    return height;
}*/