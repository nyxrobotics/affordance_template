#include "RecognitionObject.hpp"

#define PIXMAP_SIZE 100
#define OBJECT_INDEX 0
#define PACKAGE 1
#define LAUNCH_FILE 2

using namespace rviz_affordance_template_panel;

RecognitionObject::RecognitionObject(const string& object_type, const string& launch_file, const string& package, const string& image_path) {
    QPixmap pixmap(QSize(PIXMAP_SIZE,PIXMAP_SIZE));

    // set pixmap to image if it exists
    pixmap.convertFromImage(QImage(image_path.c_str()));
    if (pixmap.isNull()) {
        // otherwise set it to a green box with the class name overlayed
        QPixmap colorPixmap(QSize(PIXMAP_SIZE,PIXMAP_SIZE));
        colorPixmap.fill(Qt::green);
        QPainter text(&colorPixmap);
        QRectF rect(0, 0, PIXMAP_SIZE, PIXMAP_SIZE);
        text.drawText(rect, Qt::AlignCenter, object_type.c_str());
        this->setPixmap(colorPixmap.scaled(PIXMAP_SIZE, PIXMAP_SIZE));
    } else {
        this->setPixmap(pixmap.scaled(PIXMAP_SIZE, PIXMAP_SIZE));
    }

    this->setFlag(QGraphicsItem::ItemIsSelectable);

    // store the class name associated with the template pixmap item
    // we'll use the class name to instantiate an object template using Pluginlib
    this->setData(OBJECT_INDEX, QVariant(object_type.c_str()));
    this->setData(LAUNCH_FILE, QVariant(launch_file.c_str()));
    this->setData(PACKAGE, QVariant(package.c_str()));

    this->key_ = object_type;
    this->package_ = package;
    this->launch_file_ = launch_file;
    this->image_path_ = image_path;

}