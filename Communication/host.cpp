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
    lidar_height = 640;
    lidar_width = 640;
    traj_map_img = cv::Mat::zeros(height, width, CV_8UC3);
    map_img = cv::Mat::zeros(height, width, CV_8UC3);
    lidar_img0 = cv::Mat::zeros(lidar_height, lidar_width, CV_8UC3);
    lidar_img1 = cv::Mat::zeros(lidar_height, lidar_width, CV_8UC3);
    lastX0 = 0;lastY0 = 0;lastX1 = 0;lastY1 = 0;
    biasX = 50;biasY = 50;
    lidar_biasX = lidar_width/2;lidar_biasY = lidar_height;
    block0_size = 0;
    block1_size = 0;

    map = new float*[height];
    for( int i=0; i<height; i++ ){
        map[i] = new float[width];
        for( int j=0; j<width; j++ ){
            map[i][j] = 0;
        }
    }

    drawer = new Drawer();
    workerThread = new QThread();
    drawer->moveToThread(workerThread);
    workerThread->start();
    connect(workerThread, &QThread::finished, drawer, &QObject::deleteLater);
    connect(server0, &QTcpServer::newConnection, drawer, &Drawer::doWork);

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

void Host::UpdateMap(float x, float y, float orien,
                     const int size, const quint32 from_num, const short* data){
    /*for( int i=0; i<size; i++ )
        std::cout<<data[i]/laser_params.unit<<" ";
    std::cout<<std::endl;*/
    //qDebug()<<"data size is "<<size;
    cv::Mat lidar_img;
    if( from_num==0 )
        lidar_img = lidar_img0;
    else{
        lidar_img = lidar_img1;
        float tmp = x;x=y;y=tmp;
        orien -= 1.57;
    }
    lidar_img.setTo(125);
    int cnt = 0;
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
        //std::cout<<"("<<lx<<","<<ly<<")->";
        //if(laser_params.isReverse)
        //    lx = -lx;
        //std::cout<<"("<<lx<<","<<ly<<")->";
        double laser_al_rad = laser_params.aL * PI / 180;
        double laser_xl = laser_params.xL;
        double laser_yl = laser_params.yL;
        ry= laser_xl - lx*cos(laser_al_rad) + ly*sin(laser_al_rad) ;
        rx= laser_yl + ly*cos(laser_al_rad) + lx*sin(laser_al_rad) ;
        //std::cout<<laser_al_rad<<"->";
        //std::cout<<"("<<rx<<","<<ry<<")->";
        //qDebug()<<"rx "<<rx<<" ry "<<ry;


        gx =x + ry*cos(orien) - rx*sin(orien);
        gy =y + ry*sin(orien) + rx*cos(orien);
        //qDebug()<<"gx "<<gx<<" gy "<<gy;
        //std::cout<<orien<<"->";


        int mapx, mapy;
        mapx = gx*mapping_params.mapRes + biasX;
        mapy = gy*mapping_params.mapRes + biasY;
        //qDebug()<<"mapx "<<mapx<<" mapy "<<mapy;
        //std::cout<<"("<<gx<<","<<gy<<") ";

        //cv::waitKey(500);
        int location_mapx,location_mapy;
        //location_mapx=x/mapping_params.mapRes + biasX;
        //location_mapy=y/mapping_params.mapRes + biasY;
        location_mapx = x*(float)mapping_params.mapRes + biasX;
        location_mapy = y*(float)mapping_params.mapRes + biasY;
        std::vector<Location> locationVec;
        Location startPos(location_mapx, location_mapy);
        Location endPos(mapx,mapy);
        CalcShortestDistance(startPos, endPos, locationVec);
        /*if( cnt<10 ){
            std::cout<<"("<<x<<","<<y<<")->("<<gx<<","<<gy<<")--->";
            std::cout<<"("<<location_mapx<<","<<location_mapy<<")&"
                       <<"("<<mapx<<","<<mapy<<")"<<"     ";
            cnt++;
        }*/


        double upthres=300;
        double lowthres=-300;
        int lidar_map_x, lidar_map_y;
        //std::cout <<"get here !"<<std::endl;
        map_mutex.lock();
        for (std::vector<Location>::iterator c=locationVec.begin();c!=locationVec.end();c++){
            //(*c).x *= 60;(*c).y *= 60;
            if((*c).x >= 0 && (*c).x < mapping_params.mapWidth && (*c).y >= 0 && (*c).y < mapping_params.mapHeight){
                    if(c != locationVec.end()-1 ){
                        map[(*c).x][(*c).y] -= 0.07;
                        if( map[(*c).x][(*c).y] < lowthres )
                            map[(*c).x][(*c).y] = lowthres;

                    }
                    else{
                        map[(*c).x][(*c).y] += 0.1;
                        if( map[(*c).x][(*c).y] > upthres )
                            map[(*c).x][(*c).y] = upthres;
                    }
            }
            if( from_num==0 ){
                lidar_map_y = (*c).y - location_mapy;
                lidar_map_x = (*c).x - location_mapx;
            }
            if(from_num==1){
                lidar_map_x = (*c).y - location_mapy;
                lidar_map_y = (*c).x - location_mapx;

            }
            lidar_map_x += lidar_biasX;
            if(lidar_map_x >= 0 && lidar_map_x <= lidar_width
                    && lidar_map_y >= 0 && lidar_map_y <= lidar_height){
                if( c != locationVec.end()-1 )
                    lidar_img.at<uchar>(lidar_map_y, lidar_map_x) = 0;
                else
                    lidar_img.at<uchar>(lidar_map_y, lidar_map_x) = 255;
            }
        }
        map_mutex.unlock();
    }
    //std::cout<<std::endl;

    map_img_mutex.lock();
    if(from_num==0){
        cv::Scalar traj_color(255,0,0);
        cv::line(traj_map_img, cv::Point2f(biasX+lastX0*mapping_params.mapRes, biasY+lastY0*mapping_params.mapRes)
                 , cv::Point2f(biasX+x*mapping_params.mapRes, biasY+y*mapping_params.mapRes), traj_color, 3);
        lastX0=x;lastY0=y;
    }
    else{
        cv::Scalar traj_color(0,255,0);
        cv::line(traj_map_img, cv::Point2f(biasX+lastX1*mapping_params.mapRes, biasY+lastY1*mapping_params.mapRes)
                 , cv::Point2f(biasX+x*mapping_params.mapRes, biasY+y*mapping_params.mapRes), traj_color, 3);
        lastX1=x;lastY1=y;
    }
    map_img = traj_map_img.clone();
    for(int i=0; i<height; i++ )
        for(int j=0; j<width; j++ ){
            if(traj_map_img.at<cv::Vec3b>(j,i)==cv::Vec3b(0,0,0)){
                int num = map[i][j]*2+125;
                if(num<0)
                    num = 0;
                if(num>255)
                    num = 255;
                map_img.at<cv::Vec3b>(j,i) = cv::Vec3b(num,num,num);
            }
        }

    cv::Scalar traj_color(255,0,0);
    cv::circle(map_img, cv::Point2f(biasX+lastX0*mapping_params.mapRes, biasY+lastY0*mapping_params.mapRes)
               , 10, traj_color, 6);
    traj_color = cv::Scalar(0,255,0);
            cv::circle(map_img, cv::Point2f(biasX+lastX1*mapping_params.mapRes, biasY+lastY1*mapping_params.mapRes)
               , 10, traj_color, 6);
    map_img_mutex.unlock();

    //cv::imshow("Map", map_img);
    //cv::imshow("Lidar", lidar_img);
    //cv::waitKey(5);

}

void Host::ReadData0(){
    //qDebug()<<"ReadData bytes available "<<socket0->bytesAvailable();
    QDataStream in(socket0);
    if( block0_size==0 ){
        if( socket0->bytesAvailable() < (int)sizeof(quint16) )
            return;
        in>>block0_size;
    }
    if( socket0->bytesAvailable() < block0_size ){
        return;
    }


    qint16 data[361];
    qint64 timestamp;
    quint32 from_num;
    qint32 size;
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
               <<", message size is "<<block0_size<<", x "<<x<<", y "<<y<<", ori "<<orien
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
    //qDebug()<<"ReadData bytes available "<<socket1->bytesAvailable();
    QDataStream in(socket1);
    if( block1_size==0 ){
        if( socket1->bytesAvailable() < (int)sizeof(quint16) )
            return;
        in>>block1_size;
    }
    if( socket1->bytesAvailable() < block1_size ){
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
            block1_size = 0;
            return;
        }
        //data = new short[size];
        for( int i=0; i<size; i++ ){
            in>>data[i];
        }

        qDebug()<<"Receive message from "<<from_num<<" message type "<<message_type<<",timestamp "<<timestamp
               <<", message size is "<<block1_size<<", x "<<x<<", y "<<y<<", ori "<<orien
               <<" data size "<<size<<" bytesAvailable "<<socket1->bytesAvailable();
        //UpdateMap(x, y, orien, size, from_num, data);
        //delete data;

    }
    else{
        qDebug()<<"Message type not 0";
    }
    block1_size = 0;
}
