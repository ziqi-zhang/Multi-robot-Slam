#include "../Communication/tcpclient.h"

TcpClient::TcpClient(Network* network_)
{
    socket = new QTcpSocket();
    socket_host = new QTcpSocket();
    network = network_;
    self_num = network->self_num;
    cnt = 0;
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
    std::cout<<"1Send pose to ";
    if( self_num == mini0_num )
        std::cout<<"1"<<std::endl;
    else
        std::cout<<"0"<<std::endl;
    socket->disconnectFromHost();
    //socket->abort();

}

void TcpClient::SendMessageToHost(float x, float y, float ori, SensorTimer_Sensor_URG_Data* lidarData){

    if( self_num==mini0_num )
        socket_host->connectToHost(*(network->Host), HOSTPORT0);
    else
        socket_host->connectToHost(*(network->Host), HOSTPORT1);
    socket_host->waitForConnected();
    message_type = 1;
    QByteArray message;
    QDataStream out(&message, QIODevice::WriteOnly);
    out<<(quint16)0;
    out<<message_type;
    qint64 timestamp=lidarData->timestamp;
    out<<timestamp;
    out<<self_num;
    qint32 size = lidarData->datasize;
    out<<size;
    out<<x<<y<<ori;
    for( int i=0; i<size; i++ )
        out<<qint16(lidarData->data[i]);
    out.device()->seek(0);
    out << (quint16)(message.size() - sizeof(quint16));
    socket_host->write(message);
    socket_host->flush();
    //socket_host->waitForBytesWritten();
    //printf("Send to host %d bytes\n", message.size());
    std::cout<<"Send to host "<<message.size()<<" bytes "<<"timestamp "<<timestamp
            <<" x "<<x<<" y "<<y<<std::endl;
    socket_host->disconnectFromHost();
}

