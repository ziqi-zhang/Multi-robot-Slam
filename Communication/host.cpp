#include "host.h"
#include <unistd.h>


Host::Host(QObject *parent, Network* network_) :
    QObject(parent)
{
    network = network_;
    server0 = new QTcpServer();
    server1 = new QTcpServer();
    server0->listen(QHostAddress::Any, HOSTPORT0);
    server1->listen(QHostAddress::Any, HOSTPORT1);
    connect(server0, SIGNAL(newConnection()), this, SLOT(NewConnection0()));
    connect(server1, SIGNAL(newConnection()), this, SLOT(NewConnection1()));
    height = 640;
    width = 640;
    traj_map_img = cv::Mat::zeros(height, width, CV_8UC3);
    map_img = cv::Mat::zeros(height, width, CV_8UC3);
    lastX0 = 0;lastY0 = 0;lastX1 = 0;lastY1 = 0;
    biasX = width/2;biasY = height/2;
    init = false;
    block0_size = 0;
    block1_size = 0;

    map = new float*[height];
    for( int i=0; i<height; i++ ){
        map[i] = new float[width];
        for( int j=0; j<width; j++ ){
            map[i][j] = 0;
        }
    }
    qDebug()<<"Host constructed";
}

Host::~Host(){
    delete server0;
    delete server1;
}

void Host::NewConnection0(){
    socket0 = server0->nextPendingConnection();
    connect(socket0, SIGNAL(readyRead()), this, SLOT(ReadData0()));
}

void Host::NewConnection1(){
    socket1 = server1->nextPendingConnection();
    connect(socket1, SIGNAL(readyRead()), this, SLOT(ReadData1()));
}

void Host::DisConnection(){
    //printf("Disconnect");
    qDebug()<<"Disconnected";
}

void Host::UpdateMap(const float x, const float y, const float orien,
                     const int size, const quint32 from_num, const short* data){
    /*for( int i=0; i<size; i++ )
        std::cout<<data[i]<<" ";
    std::cout<<std::endl;*/
    //qDebug()<<"data size is "<<size;
    for( int i=0; i<size; i++ ){
        double gx, gy;//激光点在全局坐标系中的位置 单位m
        double rx, ry;//激光点在机器人坐标系中的位置 单位m
        double lx, ly;//激光点在激光雷达坐标系中的位置 单位m

        if( data[i]==0 )
            continue;

        double dis = data[i]/laser_params.unit;
        //qDebug()<<laser_params.unit;
        double ang = i * laser_params.res * PI / 180;
        //qDebug()<<"dis "<<dis<<" ang "<<ang;
        lx = dis*cos(ang);
        ly = dis*sin(ang);
        //qDebug()<<"lx "<<lx<<" ly "<<ly;
        if(laser_params.isReverse)
            lx = -lx;
        double laser_al_rad = laser_params.aL * PI / 180;
        double laser_xl = laser_params.xL;
        double laser_yl = laser_params.yL;
        rx= laser_xl - lx*cos(laser_al_rad) + ly*sin(laser_al_rad) ;
        ry= laser_yl + ly*cos(laser_al_rad) + lx*sin(laser_al_rad) ;
        //qDebug()<<"rx "<<rx<<" ry "<<ry;

        gx =x + ry*cos(orien) - rx*sin(orien);
        gy =y + ry*sin(orien) + rx*cos(orien);
        //qDebug()<<"gx "<<gx<<" gy "<<gy;

        int mapx, mapy;
        mapx = gx + biasX;
        mapy = gy + biasY;
        //qDebug()<<"mapx "<<mapx<<" mapy "<<mapy;
        std::cout<<"("<<gx<<","<<gy<<") ";

        //cv::waitKey(500);
        int location_mapx,location_mapy;
        //location_mapx=x/mapping_params.mapRes + biasX;
        //location_mapy=y/mapping_params.mapRes + biasY;
        location_mapx = x + biasX;
        location_mapy = y + biasY;
        std::vector<Location> locationVec;
        Location startPos(location_mapx, location_mapy);
        Location endPos(mapx,mapy);
        CalcShortestDistance(startPos, endPos, locationVec);


        double upthres=300;
        double lowthres=-300;
        //std::cout <<"get here !"<<std::endl;
        for (std::vector<Location>::iterator c=locationVec.begin();c!=locationVec.end();c++){
            //(*c).x *= 60;(*c).y *= 60;
            if((*c).x >= 0 && (*c).x < mapping_params.mapWidth && (*c).y >= 0 && (*c).y < mapping_params.mapHeight){
                    if(c != locationVec.end()-1 ){
                        map[(*c).y][(*c).x] -= 1;
                        if( map[(*c).y][(*c).x] < -300 )
                            map[(*c).y][(*c).x] = -300;
                    }
                    else{
                        map[(*c).y][(*c).x] += 1;
                        if( map[(*c).y][(*c).x] > 300 )
                            map[(*c).y][(*c).x] = 300;
                    }
            }
        }
    }
    std::cout<<std::endl;

    map_img = traj_map_img.clone();
    if(from_num==0){
        cv::Scalar traj_color(255,0,0);
        cv::line(traj_map_img, cv::Point2f(biasX+lastX0*60, biasY+lastY0*60), cv::Point2f(biasX+x*60, biasY+y*60),
                 traj_color, 3);
        lastX0=x;lastY0=y;
    }
    else{
        cv::Scalar traj_color(0,255,0);
        cv::line(traj_map_img, cv::Point2f(biasX+lastX1, biasY+lastY1), cv::Point2f(biasX+x, biasY+y),
                 traj_color, 1);
        lastX1=x;lastY1=y;
    }
    for(int i=0; i<height; i++ )
        for(int j=0; j<width; j++ ){
            int num = map[i][j]*5+125;
            map_img.at<cv::Vec3b>(j,i) = cv::Vec3b(num,num,num);
        }
    if(from_num==0){
        cv::Scalar traj_color(255,0,0);
        cv::circle(map_img, cv::Point2f(biasX+lastX0*60, biasY+lastY0*60), 10, traj_color, 6);
    }
    else{
        cv::Scalar traj_color(0,255,0);
        cv::circle(map_img, cv::Point2f(biasX+lastX1, biasY+lastY1), 10, traj_color, 2);
    }

    cv::imshow("Map", map_img);

    //cv::imshow("map", traj_map_img);
    cv::waitKey(5);

}

void Host::ReadData0(){
    QDataStream in(socket0);
    if( block0_size==0 ){
        if( socket0->bytesAvailable() < (int)sizeof(quint16) )
            return;
        in>>block0_size;
    }
    if( socket0->bytesAvailable() < block0_size ){
        return;
    }


    short data[361];
    long long timestamp;
    quint32 from_num;
    int size;
    float x, y, orien;
    quint32 message_type;


    in>>message_type;
    if( message_type==1 ){
        in>>timestamp>>from_num>>size>>x>>y>>orien;
        if( timestamp<0 ){
            qDebug()<<"timestamp not positive";
            block0_size = 0;
            return;
        }
        //data = new short[size];
        for( int i=0; i<size; i++ ){
            in>>data[i];
        }

        qDebug()<<"Receive message from "<<from_num<<" message type "<<message_type<<",timestamp "<<timestamp
               <<", message size is "<<block0_size<<", x "<<x<<", y "<<y
               <<" data size "<<size<<" bytesAvailable "<<socket0->bytesAvailable();
        UpdateMap(x, y, orien, size, from_num, data);
        //delete data;

    }
    else{
        qDebug()<<"Message type not 0";
    }
    block0_size = 0;

}

void Host::ReadData1(){
    QByteArray message;
    short* data;
    long long timestamp;
    quint32 from_num;
    int size;
    float x, y, orien;
    quint32 message_type;
    message = socket1->readAll();
    QDataStream in(&message, QIODevice::ReadOnly);
    in>>message_type;
    if( message_type==1 ){
        in>>timestamp>>from_num>>size>>x>>y>>orien;
        data = new short[size];
        for( int i=0; i<size; i++ ){
            in>>data[i];
        }
    }

    qDebug()<<"Receive message from "<<from_num<<", message size is "
               <<message.size()<<", x "<<x<<", y "<<y;
    UpdateMap(x, y, orien, size, from_num, data);
    delete data;
}
