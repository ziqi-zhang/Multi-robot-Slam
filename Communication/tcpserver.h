#ifndef TCPSERVER_H
#define TCPSERVER_H

#include <QObject>
#include "Tcp.h"
#include <QtNetwork>
#include <QTcpSocket>
#include <QTcpServer>

class TcpServer : public QObject
{
    Q_OBJECT
public:
    explicit TcpServer(Network* network_);
    ~TcpServer();

private:
    QTcpServer* server;
    QTcpSocket* socket;
    quint32 self_num;
    quint32 message_type;
    Network* network;


signals:

public slots:
    void NewConnection();
    void DisConnection();
    void ReadData();

};

#endif // TCPSERVER_H
