#ifndef NETWORK_H
#define NETWORK_H

#include <QObject>
#include <QtNetwork>

class Network : public QObject
{
    Q_OBJECT
public:
    explicit Network(QObject *parent = 0);

    QString ip0, ip1, ip_host;
    QHostAddress* Mini0, *Mini1, *Host;
    quint32 self_num;

signals:

public slots:

};

#endif // NETWORK_H
