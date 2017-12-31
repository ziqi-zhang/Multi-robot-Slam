#include "network.h"
#include <QFile>
#include <iostream>

Network::Network(QObject *parent) :
    QObject(parent)
{
    QFile file("/home/rsys/SDK/RobotSDK/ModuleDev/Robot/network.xml");
    if( !file.open(QFile::ReadOnly|QFile::Text) ){
        //printf("Open xml file failed:");
        qDebug()<<"Open xml file failed";
        qDebug()<<qPrintable(file.errorString());
    }
    QXmlStreamReader reader(&file);
    reader.readNext();
    while(!reader.atEnd()){
        if(reader.name()=="SelfNumber"){
            self_num = reader.readElementText().toInt();
            //printf("Read SelfNumber %d\n", self_num);
        }
        if(reader.name()=="IP0"){
            ip0 = reader.readElementText().toCaseFolded();
            Mini0 = new QHostAddress(ip0);
            //printf("Mini0 IP %s\n", ip0.toStdString().c_str());

        }
        if(reader.name()=="IP1"){
            ip1 = reader.readElementText().toCaseFolded();
            Mini1 = new QHostAddress(ip1);
            //printf("Mini1 IP %s\n", ip1.toStdString().c_str());
        }
        if(reader.name()=="IPHOST"){
            ip_host = reader.readElementText().toCaseFolded();
            Host = new QHostAddress(ip_host);
            //printf("HOST IP %s\n", ip_host.toStdString().c_str());
        }
        reader.readNext();

    }
    file.close();
}
