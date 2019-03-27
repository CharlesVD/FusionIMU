#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QTimer>

#include "sensores_imu.h"

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
    void actualizaDatos();
    void integracion();

    void on_pushButtonIniciar_clicked();

private:
    Ui::MainWindow *ui;

    QTimer *timerLectura;
    QTimer *timerIntegracion;
    Sensores_IMU i2c;

    void calcularAcelMag();

    double roll;
    double pitch;
    double yaw;

    double giroRoll;
    double giroPitch;
    double giroYaw;

    double fusionRoll;
    double fusionPitch;
    double fusionYaw;

    double giroX_t0;
    double giroY_t0;
    double giroZ_t0;

    double tiempo;
};

#endif // MAINWINDOW_H
