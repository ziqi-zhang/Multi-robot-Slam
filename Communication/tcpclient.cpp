#include "../Communication/tcpclient.h"

TcpClient::TcpClient(Network* network_)
{
    socket = new QTcpSocket();
    socket_host = new QTcpSocket();
    network = network_;
    self_num = network->self_num;
}

TcpClient::~TcpClient(){
    delete socket;
}

void TcpClient::SendPos(float x, float y){
    if( self_num == mini0_num ){
        socket->connectToHost(*(network->Mini1), PORT);
    }
    else
        socket->connectToHost(*(network->Mini0), PORT);
    socket->waitForConnected();
    message_type = 0;

    QByteArray message;
    QDataStream out(&message, QIODevice::WriteOnly);
    out<<message_type<<x<<y;
    socket->write(message);
    socket->flush();
    //std::cout<<"sender "<<socket->bytesToWrite();
    std::cout<<"Send pose to ";
    if( self_num == mini0_num )
        std::cout<<"1"<<std::endl;
    else
        std::cout<<"0"<<std::endl;
    socket->disconnectFromHost();
    //socket->abort();

}

void TcpClient::SendLidarMessage(SensorTimer_Sensor_URG_Data* lidarData){
    if( self_num==mini0_num )
        socket_host->connectToHost(*(network->Host), HOSTPORT0);
    else
        socket_host->connectToHost(*(network->Host), HOSTPORT1);
    socket_host->waitForConnected();
    message_type = 1;
    QByteArray message;
    QDataStream out(&message, QIODevice::WriteOnly);
    out<<message_type;
    out<<self_num;
    int size = lidarData->datasize;
    out<<size;
    for( int i=0; i<size; i++ )
        out<<lidarData->data[i];
    socket_host->write(message);
    socket_host->flush();
    socket_host->disconnectFromHost();
}

