#ifndef DRIVER_H
#define DRIVER_H

#include <QObject>
#include <QUdpSocket>
#include <QtSerialPort/QSerialPort>

class Driver : public QObject
{
    Q_OBJECT
public:
    explicit Driver(QObject *parent = 0);

signals:

public slots:

private:
    QUdpSocket udp_socket;
    QSerialPort serial;
};

#endif // DRIVER_H
