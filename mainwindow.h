#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>
#include <QUdpSocket>
#include <QHostAddress>
#include <QDebug>
#include <QtCore>
#include <QtNetwork>

#include <string.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void Kine_Calc_Send();

private:
    Ui::MainWindow *ui;
    QUdpSocket *ControlBrd;
    QByteArray SendDatagram, RecvData;

};

#endif // MAINWINDOW_H
