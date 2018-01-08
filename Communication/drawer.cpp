#include "drawer.h"

Drawer::Drawer()
{
    height = 640;
    width = 640;
    lidar_height = 640;
    lidar_width = 640;
    traj_map_img = cv::Mat::zeros(height, width, CV_8UC3);
    map_img = cv::Mat::zeros(height, width, CV_8UC3);
    lidar_img0 = cv::Mat::zeros(lidar_height, lidar_width, CV_8UC3);
    lidar_img1 = cv::Mat::zeros(lidar_height, lidar_width, CV_8UC3);
    lastX0 = 0;lastY0 = 0;lastX1 = 0;lastY1 = 0;
    biasX = 50;biasY = 50;
    lidar_biasX = lidar_width/2;lidar_biasY = lidar_height;

    qDebug()<<"Drawer constructed";
}
void Drawer::doWork(){
    qDebug()<<"Drawer do a work";
}
