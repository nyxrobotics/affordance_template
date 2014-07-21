#ifndef AFFORDANCE_HPP
#define AFFORDANCE_HPP

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
    class Affordance : public QGraphicsPixmapItem
    {
    public:
        Affordance(const string& class_type, const string& image_path, QMap<QString, QVariant> &waypoint_map);
        ~Affordance() {}
        string key() const { return key_; }
        QMap<QString, QVariant> map() const { return map_; }

    private:
        string key_;
        QMap<QString, QVariant> map_;
    };
}

#endif