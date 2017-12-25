#ifndef HOST_H
#define HOST_H

#include <QObject>
#include <QTcpServer>
#include "Tcp.h"

class Host : public QObject
{
    Q_OBJECT
public:
    explicit Host(QObject *parent = 0, Network* network_=NULL);
    ~Host();

signals:

public slots:
    void NewConnection(QTcpServer* server, QTcpSocket* socket);
    void DisConnection();
    void ReadData(QTcpSocket* socket);

private:
    QTcpServer* server0, *server1;
    QTcpSocket* socket0, *socket1;
    quint32 messaga_type;
    Network* network;

};

#endif // HOST_H
