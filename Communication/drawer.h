#ifndef DRAWER_H
#define DRAWER_H

#include <QObject>
#include <QDebug>
#include <QMutex>
#include "Simulator_Sensor_laser_ParamsData.h"
#include "VisualizationMulti_Show_LocationAndMap_ParamsData.h"
#include "Bresenham.h"
#include "mapimg.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>


class Drawer : public QObject
{
    Q_OBJECT
public:
    Drawer(MapImg* map_img_, int num_);

private:
    Simulator_Sensor_laser_Params laser_params;
    VisualizationMulti_Show_LocationAndMap_Params mapping_params;
    float lastX0, lastY0, lastX1, lastY1, biasX, biasY, lidar_biasX, lidar_biasY;
    int height, width, lidar_height, lidar_width;
    float **map;
    MapImg* map_img;
    int num;
    QMutex data_mutex;

    float x, y, orien;
    int size;
    short* data;

signals:

public slots:
    void drawImg();
    void run();
    void setMessage(float x_, float y_, float orien_,
                    const int size_, const short* data_);

};

#endif // DRAWER_H
