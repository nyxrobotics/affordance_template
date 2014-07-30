#ifndef RECOGNITION_OBJECT_HPP
#define RECOGNITION_OBJECT_HPP

// qt
#include <QGraphicsPixmapItem>
#include <QPixmap>
#include <QImage>
#include <QPainter>
#include <QRectF>
#include <QVariant>
#include <QMap>

#include <iostream>

using namespace std;

namespace rviz_affordance_template_panel
{
    class RecognitionObject : public QGraphicsPixmapItem
    {
    public:
        RecognitionObject(const string& object_type, const string& launch_file, const string& package, const string& image_path);
        ~RecognitionObject() {}
        string key() const { return key_; }

    private:
        string key_;
        string package_;
        string launch_file_;
        string image_path_;
    };
}

#endif