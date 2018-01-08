#ifndef HOST_H
#define HOST_H

#include <QObject>
#include <QTcpServer>
#include <QMutex>
#include "Tcp.h"
#include "Simulator_Sensor_laser_ParamsData.h"
#include "VisualizationMulti_Show_LocationAndMap_ParamsData.h"
#include "Bresenham.h"
#include "drawer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>

#define PI 3.1415926

class Host : public QObject
{
    Q_OBJECT
public:
    cv::Mat traj_map_img, map_img, lidar_img1, lidar_img0;
    Drawer* drawer;
    QThread* workerThread;
    explicit Host(QObject *parent = 0, Network* network_=NULL);
    ~Host();

signals:

public slots:
    void NewConnection0();
    void NewConnection1();
    void DisConnection();
    void ReadData0();
    void ReadData1();
    void UpdateMap(const float x, const float y, const float orien,
                   const int size, const quint32 from_num, const short* data);

private:
    QTcpServer* server0, *server1;
    QTcpSocket* socket0, *socket1;
    quint32 messaga_type;
    Network* network;
    Simulator_Sensor_laser_Params laser_params;
    VisualizationMulti_Show_LocationAndMap_Params mapping_params;
    float lastX0, lastY0, lastX1, lastY1, biasX, biasY, lidar_biasX, lidar_biasY;
    quint16 block0_size, block1_size;
    int height, width, lidar_height, lidar_width;
    float **map;
    bool init;
    QMutex map_mutex, map_img_mutex;
};

#endif // HOST_H
