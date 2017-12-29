#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <QObject>
#include <QtNetwork>
#include <QTcpSocket>
#include "Tcp.h"
#include "Sensor/URG/SensorTimer/Edit/SensorTimer_Sensor_URG_ParamsData.h"

class TcpClient : public QObject
{
    Q_OBJECT
public:
    explicit TcpClient(Network* network_);
    ~TcpClient();
    void SendPos(float x, float y);
    void SendMessageToHost(float x, float y, float ori, SensorTimer_Sensor_URG_Data* lidarData);

private:
    QTcpSocket* socket, *socket_host;
    quint32 self_num;
    quint32 message_type;
    Network* network;


signals:

public slots:

};

#endif // TCPCLIENT_H
