#ifndef DRAWER_H
#define DRAWER_H

#include <QObject>
#include <QDebug>
#include <QMutex>
#include "Simulator_Sensor_laser_ParamsData.h"
#include "VisualizationMulti_Show_LocationAndMap_ParamsData.h"
#include "Bresenham.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>

class Drawer : public QObject
{
    Q_OBJECT
public:
    Drawer();

    cv::Mat traj_map_img, map_img, lidar_img1, lidar_img0;

private:
    Simulator_Sensor_laser_Params laser_params;
    VisualizationMulti_Show_LocationAndMap_Params mapping_params;
    float lastX0, lastY0, lastX1, lastY1, biasX, biasY, lidar_biasX, lidar_biasY;
    quint16 block0_size, block1_size;
    int height, width, lidar_height, lidar_width;
    float **map;
    QMutex map_mutex, map_img_mutex;

signals:

public slots:
    void doWork();

};

#endif // DRAWER_H
