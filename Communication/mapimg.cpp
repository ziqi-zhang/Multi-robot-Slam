#include "mapimg.h"

MapImg::MapImg(){
    height = 640;
    width = 640;
    mapRes = 10;
    lidarRes = 100;
    lidar_height = 800;
    lidar_width = 800;
    traj_map_img = cv::Mat::zeros(height, width, CV_8UC3);
    map_img = cv::Mat::zeros(height, width, CV_8UC3);
    lidar_img0 = cv::Mat::zeros(lidar_height, lidar_width, CV_8UC3);
    lidar_img1 = cv::Mat::zeros(lidar_height, lidar_width, CV_8UC3);
    lastX0 = 0;lastY0 = 0.5;lastX1 = 0.5;lastY1 = 0;
    biasX = width/2-200;biasY = height/2;
    lidar_biasX = lidar_width/2;lidar_biasY = 0;

    map = new float*[height];
    for( int i=0; i<height; i++ ){
        map[i] = new float[width];
        for( int j=0; j<width; j++ ){
            map[i][j] = 0;
        }
    }
    qDebug()<<"Map image constructed";
}

void MapImg::updateMap(float x, float y, int from_num){
    map_img_mutex.lock();
    if(from_num==0){
        cv::Scalar traj_color(255,0,0);
        cv::line(traj_map_img, cv::Point2f(biasX+lastX0*mapRes, biasY+lastY0*mapRes)
                 , cv::Point2f(biasX+x*mapRes, biasY+y*mapRes), traj_color, 3);
        lastX0=x;lastY0=y;
    }
    else{
        cv::Scalar traj_color(0,255,0);
        cv::line(traj_map_img, cv::Point2f(biasX+lastX1*mapRes, biasY+lastY1*mapRes)
                 , cv::Point2f(biasX+x*mapRes, biasY+y*mapRes), traj_color, 3);
        lastX1=x;lastY1=y;
    }
    map_img = traj_map_img.clone();
    for(int i=0; i<height; i++ )
        for(int j=0; j<width; j++ ){
            if(traj_map_img.at<cv::Vec3b>(j,i)==cv::Vec3b(0,0,0)){
                int num = -map[i][j]*3+125;
                if(num<0)
                    num = 0;
                if(num>255)
                    num = 255;
                map_img.at<cv::Vec3b>(j,i) = cv::Vec3b(num,num,num);
            }
        }

    cv::Scalar traj_color(255,0,0);
    cv::circle(map_img, cv::Point2f(biasX+lastX0*mapRes, biasY+lastY0*mapRes)
               , 5, traj_color, 5);
    traj_color = cv::Scalar(0,255,0);
    cv::circle(map_img, cv::Point2f(biasX+lastX1*mapRes, biasY+lastY1*mapRes)
               , 5, traj_color,5);

    //cv::imshow("map", map_img);
    //cv::waitKey(2);

    //qDebug()<<"Update map from "<<from_num;


    cv::imwrite("/home/rsys/Desktop/map.jpg", map_img);
    map_img_mutex.unlock();
}
