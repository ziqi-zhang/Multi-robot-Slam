#include "calcuthread.h"

CalcuThread::CalcuThread(MapImg* map_img_, int num_)
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
    init_num=0;init_ori=0;

    qDebug()<<"CalcuThread "<<num<<" constructed";
}

void CalcuThread::setMessage(float x_, float y_, float orien_,
                        const int size_, const short* data_, qint64 timestamp_){
    x = x_;y = y_;
    orien = orien_;
    if((fabs(x)>0.01||fabs(y)>0.01) && orien!=0){
        if(init_num<10){
            qDebug()<<"Orien "<<orien;
            init_ori+=orien;
            init_num++;
        }
        else if(init_num==10){
            init_ori/=10;
            init_num++;
            qDebug()<<"Init orien "<<init_ori;
        }
    }
    if( num==0 ){

    }
    else{
        //float tmp = x;x=y;y=tmp;
        y = -y;
        orien -= 3.14;
    }
    size = size_;
    for(int i=0; i<size; i++)
        data[i] = data_[i];
    timestamp = timestamp_;


    //std::cout<<"set message"<<std::endl;
}

void CalcuThread::run(){
    if(init_num>=10)
        calcuMap();
}

void CalcuThread::calcuMap(){
    //std::cout<<"before="<<orien<<" ";
    if(init_num>10)
        orien-=init_ori;
    //std::cout<<" after="<<orien<<"  ";
    std::cout<<"x="<<x<<"  y="<<y;
    std::vector<Location> end_point_vec, pass_point_vec, lidar_point_vec;
    for( int i=0; i<size; i++ ){
        double gx, gy;//激光点在全局坐标系中的位置 单位m
        double rx, ry;//激光点在机器人坐标系中的位置 单位m
        double lx, ly;//激光点在激光雷达坐标系中的位置 单位m

        if( data[i]==0 )
            continue;
        //if(i<10)
        //    std::cout<<data[i]<<"  ";
        if(data[i]>600)
            data[i] = 600;
        double dis = data[i]/laser_params.unit;
        //qDebug()<<laser_params.unit;
        double ang = i * laser_params.res * PI / 180;
        //qDebug()<<"dis "<<dis<<" ang "<<ang;
        lx = dis*cos(ang);
        ly = dis*sin(ang);
        //qDebug()<<"lx "<<lx<<" ly "<<ly;
        //std::cout<<"("<<lx<<","<<ly<<")->";
        if(laser_params.isReverse && num==0)
            lx = -lx;
        //std::cout<<"("<<lx<<","<<ly<<")->";
        double laser_al_rad = laser_params.aL * PI / 180;
        double laser_xl = laser_params.xL;
        double laser_yl = laser_params.yL;
        rx= laser_xl - lx*cos(laser_al_rad) + ly*sin(laser_al_rad) ;
        ry= laser_yl + ly*cos(laser_al_rad) + lx*sin(laser_al_rad) ;
        //std::cout<<laser_al_rad<<"->";
        //std::cout<<"("<<rx<<","<<ry<<")->";
        //qDebug()<<"rx "<<rx<<" ry "<<ry;
        int lidar_location_mapx, lidar_location_mapy, lidar_mapx, lidar_mapy;
        lidar_location_mapx = rx*(float)map_img->lidarRes + map_img->lidar_biasX;
        lidar_location_mapy = ry*(float)map_img->lidarRes + map_img->lidar_biasY;
        lidar_mapx = map_img->lidar_biasX;
        lidar_mapy = map_img->lidar_biasY;
        std::vector<Location> lidarLocationVec;
        Location lidarStartPos(lidar_location_mapx, lidar_location_mapy);
        Location lidarEndPos(lidar_mapx,lidar_mapy);
        CalcShortestDistance(lidarStartPos, lidarEndPos, lidarLocationVec);
        for (std::vector<Location>::iterator c=lidarLocationVec.begin();c!=lidarLocationVec.end();c++){
            if((*c).x >= 0 && (*c).x < map_img->lidar_width&& (*c).y >= 0 && (*c).y < map_img->lidar_height){
                    lidar_point_vec.push_back((*c));
            }
        }
        //std::cout<<"("<<lidar_location_mapx<<","<<lidar_location_mapy<<")";

        double tmp = ry;
        ry=rx;rx=tmp;

        gx =x + ry*cos(orien) - rx*sin(orien);
        gy =y + ry*sin(orien) + rx*cos(orien);
        //qDebug()<<"gx "<<gx<<" gy "<<gy;
        //std::cout<<orien<<"->";


        int mapx, mapy;
        mapx = gx*map_img->mapRes + map_img->biasX;
        mapy = gy*map_img->mapRes + map_img->biasY;
        //qDebug()<<"mapx "<<mapx<<" mapy "<<mapy;


        int location_mapx,location_mapy;
        //location_mapx=x/mapping_params.mapRes + biasX;
        //location_mapy=y/mapping_params.mapRes + biasY;
        location_mapx = x*(float)map_img->mapRes + map_img->biasX;
        location_mapy = y*(float)map_img->mapRes + map_img->biasY;
        std::vector<Location> locationVec;
        Location startPos(location_mapx, location_mapy);
        Location endPos(mapx,mapy);
        CalcShortestDistance(startPos, endPos, locationVec);
        for (std::vector<Location>::iterator c=locationVec.begin();c!=locationVec.end();c++){
            if((*c).x >= 0 && (*c).x < map_img->width && (*c).y >= 0 && (*c).y < map_img->height){
                    if(c != locationVec.end()-1 ){
                        pass_point_vec.push_back((*c));
                    }
                    else{
                        end_point_vec.push_back((*c));
                    }
            }
        }
    }
    std::cout<<std::endl;

    int location_mapx,location_mapy;
    location_mapx = x*(float)map_img->mapRes + map_img->biasX;
    location_mapy = y*(float)map_img->mapRes + map_img->biasY;
    QMutex* lidar_mutex;
    if(num==0){
        lidar_mutex = &map_img->lidar0_img_mutex;
        lidar_img = map_img->lidar_img0;
    }
    else{
        lidar_mutex = &map_img->lidar1_img_mutex;
        lidar_img = map_img->lidar_img1;
    }
    int lidar_map_x, lidar_map_y;
    lidar_mutex->lock();
    lidar_img.setTo(cv::Vec3b(0,0,0));
    for (std::vector<Location>::iterator c=lidar_point_vec.begin();c!=lidar_point_vec.end();c++){
        if((*c).x >= 0 && (*c).x < map_img->lidar_width && (*c).y >= 0 && (*c).y < map_img->lidar_height){
            lidar_map_y = (*c).y;
            lidar_map_x = (*c).x;
            lidar_img.at<cv::Vec3b>(lidar_map_y, lidar_map_x) = cv::Vec3b(255,255,255);
        }
    }
    lidar_mutex->unlock();


    //std::cout<<"End point number "<<end_point_vec.size()
    //         <<"  Pass point number "<<pass_point_vec.size()<<std::endl;
    map_img->map_array_mutex.lock();
    double upthres=300;
    double lowthres=-300;

    for (std::vector<Location>::iterator c=pass_point_vec.begin();c!=pass_point_vec.end();c++){
        if((*c).x >= 0 && (*c).x < map_img->width && (*c).y >= 0 && (*c).y < map_img->height){
            map_img->map[(*c).x][(*c).y] -= 0.7;
            if( map_img->map[(*c).x][(*c).y] < lowthres )
                map_img->map[(*c).x][(*c).y] = lowthres;
        }
    }

    for (std::vector<Location>::iterator c=end_point_vec.begin();c!=end_point_vec.end();c++){
        if((*c).x >= 0 && (*c).x < map_img->width && (*c).y >= 0 && (*c).y < map_img->height){
            map_img->map[(*c).x][(*c).y] += 1;
            if( map_img->map[(*c).x][(*c).y] > upthres )
                map_img->map[(*c).x][(*c).y] = upthres;
        }
    }
    map_img->map_array_mutex.unlock();
    //qDebug()<<"Timestamp "<<timestamp<<" Calculate map once";


    map_img->updateMap(x, y, num);
    //msleep(30);


}
