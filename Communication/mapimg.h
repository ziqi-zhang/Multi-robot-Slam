#ifndef MAPIMG_H
#define MAPIMG_H

#include <QObject>
#include <QMutex>
#include <QDebug>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>

class MapImg : public QObject
{
    Q_OBJECT
public:
    explicit MapImg();

    cv::Mat traj_map_img, map_img, lidar_img1, lidar_img0;
    float lastX0, lastY0, lastX1, lastY1, biasX, biasY, lidar_biasX, lidar_biasY;
    int height, width, lidar_height, lidar_width;
    float mapRes, lidarRes;
    QMutex map_array_mutex, map_img_mutex, lidar0_img_mutex, lidar1_img_mutex;
    float **map;

signals:

public slots:
    void updateMap(float x, float y, int from_num);

};

#endif // MAPIMG_H
