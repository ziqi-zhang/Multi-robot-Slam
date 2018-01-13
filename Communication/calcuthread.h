#ifndef CALCUTHREAD_H
#define CALCUTHREAD_H

#include <QThread>
#include "mapimg.h"
#include "Tcp.h"
#include "Simulator_Sensor_laser_ParamsData.h"
#include "VisualizationMulti_Show_LocationAndMap_ParamsData.h"
#include "Bresenham.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>

#define PI 3.1415926

class CalcuThread : public QThread
{
    Q_OBJECT
public:
    explicit CalcuThread(MapImg* map_img_, int num_);


    Simulator_Sensor_laser_Params laser_params;
    VisualizationMulti_Show_LocationAndMap_Params mapping_params;
    float lastX0, lastY0, lastX1, lastY1, biasX, biasY, lidar_biasX, lidar_biasY;
    int height, width, lidar_height, lidar_width;
    cv::Mat lidar_img;
    MapImg* map_img;
    int num;
    QMutex data_mutex;

    float x, y, orien;
    int size;
    short data[361];
    qint64 timestamp;
    float init_ori;
    int init_num;

signals:

public slots:
    void calcuMap();
    void run();
    void setMessage(float x_, float y_, float orien_,
                    const int size_, const short* data_, qint64 timestamp_);

};

#endif // CALCUTHREAD_H
