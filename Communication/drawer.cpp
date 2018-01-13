#include "drawer.h"

Drawer::Drawer(MapImg* map_img_, int num_)
{
    map_img = map_img_;
    num = num_;
    height = map_img->height;
    width = map_img->width;
    lidar_height = map_img->lidar_height;
    lidar_width = map_img->lidar_width;
    lastX0 = map_img->lastX0;lastY0 = map_img->lastY0;lastX1 = map_img->lastX1;lastY1 = map_img->lastY1;
    biasX = map_img->biasX;biasY = map_img->biasY;
    lidar_biasX = lidar_width/2;lidar_biasY = lidar_height;

    qDebug()<<"Drawer constructed";
}
void Drawer::setMessage(float x_, float y_, float orien_,
                        const int size_, const short* data_){
    x = x_;y = y_;
    orien = orien_;
    size = size_;
    data = new short[size];
    for(int i=0; i<size; i++)
        data[i] = data_[i];

}

void Drawer::drawImg(){


    qDebug()<<"Drawer do a work";
}
