#include "tcpserver.h"
#include <iostream>

TcpServer::TcpServer(Network* network_)
{
    network = network_;
    self_num = network->self_num;
    server = new QTcpServer();
    server->listen(QHostAddress::Any, PORT);
    server->setMaxPendingConnections(1);
    connect(server, SIGNAL(newConnection()), this, SLOT(NewConnection()));
    std::cout<<"Build TcpServer"<<std::endl;

}

TcpServer::~TcpServer(){
    delete server;
    std::cout<<"Delete TcpServer"<<std::endl;
}

void TcpServer::NewConnection(){
    socket = server->nextPendingConnection();


    connect(socket, SIGNAL(readyRead()), this, SLOT(ReadData()));
    connect(socket, SIGNAL(disconnected()), this, SLOT(DisConnection()));
    std::cout<<"TcpServer have new connection"<<std::endl;
    std::cout<<socket->isReadable()<<" "<<socket->bytesAvailable()<<std::endl;
}

void TcpServer::DisConnection(){
    //printf("Disconnect");
    std::cout<<"TcpServer Disconnect"<<std::endl;
}

void TcpServer::ReadData(){
    std::cout<<"Read data"<<std::endl;
    QByteArray data;
    data = socket->readAll();
    QDataStream in(&data, QIODevice::ReadOnly);
    in>>message_type;
    if( message_type==0 ){
        float x, y;
        in>>x>>y;
        //printf("Receive position from Mini");

        if( self_num==mini0_num ){
            qDebug()<<"Receive position from Mini1 "<<"("<<x<<","<<y<<")";
        }
        else{
            printf("0");
            qDebug()<<"Receive position from Mini0 "<<"("<<x<<","<<y<<")";
        }

    }

}
