#include "Communication/host.h"


Host::Host(QObject *parent, Network* network_) :
    QObject(parent)
{
    network = network_;
    server0 = new QTcpServer();
    server1 = new QTcpServer();
    server0->listen(QHostAddress::Any, HOSTPORT0);
    server1->listen(QHostAddress::Any, HOSTPORT1);
    connect(server0, SIGNAL(newConnection()), this, SLOT(NewConnection(server0, socket0)));
    connect(server1, SIGNAL(newConnection()), this, SLOT(NewConnection(server1, socket1)));
}

Host::~Host(){
    delete server0;
    delete server1;
}

void Host::NewConnection(QTcpServer* server, QTcpSocket* socket){
    socket = server->nextPendingConnection();
    connect(socket, SIGNAL(readyRead()), this, SLOT(ReadData(socket)));
}

void Host::DisConnection(){
    printf("Disconnect");
}

void Host::ReadData(QTcpSocket* socket){
    QByteArray message;
    short* data;
    int from_num, size;
    quint32 message_type;
    message = socket->readAll();
    QDataStream in(&message, QIODevice::ReadOnly);
    in>>message_type;
    if( message_type==1 ){
        in>>from_num>>size;
        data = new short[size];
        for( int i=0; i<size; i++ ){
            in>>data[i];
        }
    }
    printf("Receive lidar message from %d\n", from_num);


}
