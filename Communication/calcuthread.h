#ifndef CALCUTHREAD_H
#define CALCUTHREAD_H

#include <QThread>

class CalcuThread : public QThread
{
    Q_OBJECT
public:
    explicit CalcuThread(QObject *parent = 0);

signals:

public slots:

};

#endif // CALCUTHREAD_H
