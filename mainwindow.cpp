#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "Algorithm.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //2018-9-18 22:56
    ControlBrd=new QUdpSocket(this);
    ControlBrd->bind(QHostAddress("192.168.1.13"),6667);//
    connect(ControlBrd, SIGNAL(readyRead()), this, SLOT(Kine_Calc_Send()) );
    //2018-9-18 22:56
}

MainWindow::~MainWindow()
{
    delete ui;
}

//2018-9-20 22:56
void MainWindow::Kine_Calc_Send()
{
    quint8 UDP_Unicast_SendData[3*3*8]={0}, UDP_Unicast_RecvData[4*3*8]={0};
    Eigen::MatrixXd ang_real(3,2), ang_targ(3,2);

    quint8 i,j;
    Eigen::Matrix3d mtor_value = Eigen::MatrixXd::Zero(3,3);
    qint64 N_size = 0;

    if(ControlBrd->hasPendingDatagrams())
    {
        /*read alpha_real, belta_real, alpha_desi, belta_desi,defo_tendon1,2,3 from UDP data,*/
        N_size = ControlBrd->pendingDatagramSize();
        RecvData.resize(N_size);

        ControlBrd->readDatagram(RecvData.data(), RecvData.size());
        memcpy( &UDP_Unicast_RecvData,RecvData.data(),RecvData.size() );

        for(i = 0; i < 3; i++)
        {
            for(j = 0; j < 2; j++)
            {
                ang_real(i, j) = *(double *) (UDP_Unicast_RecvData+(2*i + j)*8);
            }
        }

        for(i = 0; i < 3; i++)
        {
            for(j = 0; j < 2; j++)
            {
                ang_targ(i, j) = *(double *) (UDP_Unicast_RecvData+(2*i + j + 6)*8);
            }
        }

        /*and invoke motor_actua_values() to caculate motors' actuation values*/
        mtor_value = algorithm(ang_real, ang_targ);

        /*send to F407 via UDP*/
        //convert Vector3d to QbyteArray
        for(i = 0; i < 3; i++)
        {
            for(j = 0; j < 3; j++)
            {
                *(double *) ( UDP_Unicast_SendData+(3*i+j)*8 )= mtor_value(i ,j);
            }
        }

        SendDatagram.resize(3*3*8);
        memcpy(SendDatagram.data(),UDP_Unicast_SendData,sizeof(UDP_Unicast_SendData));
        ControlBrd->writeDatagram(SendDatagram, QHostAddress("192.168.1.251"), 1031);
    }

}
//2018-9-18 22:56
