#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    giroRoll=0;
    giroPitch=0;
    giroYaw=0;

    fusionRoll=0;
    fusionPitch=0;
    fusionYaw=0;

    giroX_t0=0;
    giroY_t0=0;
    giroZ_t0=0;

    roll=0;
    pitch=0;
    yaw=0;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::calcularAcelMag()
{
    roll= ((float)atan2(i2c.acelY(), i2c.acelZ()))* 180 / PI_F;

    if (i2c.acelY() * sin(roll) + i2c.acelZ() * cos(roll) == 0)
    {
        pitch = i2c.acelX() > 0 ? (PI_F / 2) : (-PI_F / 2);
    }
    else
    {
        pitch = ((float)atan(-i2c.acelX() / (i2c.acelY() * sin(roll) + i2c.acelZ() * cos(roll))))* 180 / PI_F;
    }

    yaw = ((float)atan2( i2c.magZ() * sin(roll) - i2c.magY() * cos(roll), i2c.magX() * cos(pitch) + i2c.magY() * sin(pitch) * sin(roll) + i2c.magZ() * sin(pitch) * cos(roll)))* 180 / PI_F;
}

void MainWindow::on_pushButtonIniciar_clicked()
{
    QMessageBox::warning(this,"Aviso","Se comenzará a calibrar los sensores, no mover.",QMessageBox::Ok);
    ui->statusBar->showMessage("Calibrando giroscopio");
    i2c.calibrarGiro(5000);

    i2c.leerSensores();
    calcularAcelMag();

    timerLectura = new QTimer(this);
    connect(timerLectura,SIGNAL(timeout()),this,SLOT(actualizaDatos()));
    timerLectura->start();

    giroRoll=roll;
    giroPitch=pitch;
    giroYaw=yaw;

    giroX_t0=roll;
    giroY_t0=pitch;
    giroZ_t0=yaw;

    tiempo = 0.1;
    timerIntegracion = new QTimer(this);
    connect(timerIntegracion,SIGNAL(timeout()),this,SLOT(integracion()));
    timerIntegracion->setInterval(tiempo*1000);
    timerIntegracion->start();
}

void MainWindow::actualizaDatos()
{
    i2c.leerSensores();
    ui->doubleSpinBoxGyroX->setValue(i2c.gyroX());
    ui->doubleSpinBoxGyroY->setValue(i2c.gyroY());
    ui->doubleSpinBoxGyroZ->setValue(i2c.gyroZ());

    ui->doubleSpinBoxAcelX->setValue(i2c.acelX());
    ui->doubleSpinBoxAcelY->setValue(i2c.acelY());
    ui->doubleSpinBoxAcelZ->setValue(i2c.acelZ());

    ui->doubleSpinBoxMagX->setValue(i2c.magX());
    ui->doubleSpinBoxMagY->setValue(i2c.magY());
    ui->doubleSpinBoxMagZ->setValue(i2c.magZ());

    double gravedad = sqrt(pow(i2c.acelX(),2)+pow(i2c.acelY(),2)+pow(i2c.acelZ(),2));
    double magTierra = sqrt(pow(i2c.magX(),2)+pow(i2c.magY(),2)+pow(i2c.magZ(),2));

    ui->doubleSpinBoxGravedad->setValue( gravedad );
    ui->doubleSpinBoxMagTierra->setValue( magTierra );

    calcularAcelMag();

    ui->doubleSpinBoxRoll->setValue(roll);
    ui->doubleSpinBoxPitch->setValue(pitch);
    ui->doubleSpinBoxYaw->setValue(yaw);


    fusionRoll = roll*0.8 + giroRoll*0.2;
    fusionPitch = pitch*0.8 + giroPitch*0.2;
    fusionYaw = yaw*0.05 + giroYaw*0.95;

    ui->doubleSpinBoxFusionRoll->setValue(fusionRoll);
    ui->doubleSpinBoxFusionPitch->setValue(fusionPitch);
    ui->doubleSpinBoxFusionYaw->setValue(fusionYaw);
}

void MainWindow::integracion()
{
    giroRoll  = fusionRoll + ((giroX_t0+i2c.gyroX())/2)*tiempo;
    giroPitch = fusionPitch+ ((giroY_t0+i2c.gyroY())/2)*tiempo;
    giroYaw+=  ((giroZ_t0+i2c.gyroZ())/2)*tiempo;

    giroX_t0=i2c.gyroX();
    giroY_t0=i2c.gyroY();
    giroZ_t0=i2c.gyroZ();

    ui->doubleSpinBoxGiroRoll->setValue(giroRoll);
    ui->doubleSpinBoxGiroPitch->setValue(giroPitch);
    ui->doubleSpinBoxGiroYaw->setValue(giroYaw);
}
