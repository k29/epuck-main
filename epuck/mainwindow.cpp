#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "cameraworker.h"
#include <QThread>
#include <QDebug>
#include "time.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    cameraThread = new QThread;
    camMutex = new QMutex;
    bsMutex = new QMutex;
    bs = NULL;

    cw = new CameraWorker(cameraThread, camMutex, &bs, bsMutex);
    qRegisterMetaType<PointList>("PointList");
    qRegisterMetaType<Circle>("Circle");
    cw->moveToThread(cameraThread);
//    connect(cw, SIGNAL(error(QString)), this, SLOT(errorString(QString)));
    connect(cameraThread, SIGNAL(started()), cw, SLOT(process()));
    connect(cw, SIGNAL(finished()), cameraThread, SLOT(quit()));
    connect(cw, SIGNAL(finished()), cw, SLOT(deleteLater()));
    connect(cameraThread, SIGNAL(finished()), cameraThread, SLOT(deleteLater()));

    connect(this, SIGNAL(stopCamThread()), cw, SLOT(onStop()));


    connect(cw, SIGNAL(imageReady(QPixmap*)), this, SLOT(onCamImageReady(QPixmap*)));

    cameraThread->start(QThread::TimeCriticalPriority);

    algoThread = new QThread;
    aw = new AlgoWorker(algoThread, &bs, bsMutex);
    aw->moveToThread(algoThread);

    connect(algoThread, SIGNAL(started()), aw, SLOT(process()));
    connect(aw, SIGNAL(finished()), algoThread, SLOT(quit()));
    connect(aw, SIGNAL(finished()), aw, SLOT(deleteLater()));
    connect(algoThread, SIGNAL(finished()), algoThread, SLOT(deleteLater()));

    connect(this, SIGNAL(stopAlgoThread()), aw, SLOT(onStop()));
    connect(this, SIGNAL(algoChanged(int)), aw, SLOT(onAlgoChanged(int)));
    connect(this, SIGNAL(algoActivationChanged(int)), aw, SLOT(onAlgoActivationChanged(int)));
    connect(this, SIGNAL(resetClicked(int)), aw, SLOT(onResetClicked(int)));
    connect(aw, SIGNAL(gotResult(QString)), this, SLOT(onGetResult(QString)));

    connect(this, SIGNAL(stopAlgo()), aw, SLOT(onStopAlgo()));

    algoThread->start(QThread::TimeCriticalPriority);


    connect(aw, SIGNAL(gotLine(int, int, int, int)), cw, SLOT(onGotLine(int, int, int, int)));
    connect(aw, SIGNAL(printDestination(PointList)), cw, SLOT(onPrintDestination(PointList)));
    connect(aw, SIGNAL(printCircle(Circle)), cw, SLOT(onPrintCircle(Circle)));

    getfps.start();

}

void MainWindow::onCamImageReady(QPixmap *pm)
{


    double secondsElapsed = getfps.elapsed();
//    qDebug() << secondsElapsed;
    ui->fpsLabel->setText(QString::number(1000/secondsElapsed));
    getfps.restart();
    camMutex->lock();
    ui->imgLabel->setPixmap(*pm);
    camMutex->unlock();

//    bsMutex->lock();

//    qDebug() << bs->angle;

//    bsMutex->unlock();
}



MainWindow::~MainWindow()
{
    emit stopCamThread();
    cameraThread->wait();
    emit stopAlgoThread();
    algoThread->wait();
    delete aw;
    delete cw;
    delete ui;
}


void MainWindow::on_algoStopButton_clicked()
{
    emit stopAlgo();
}

void MainWindow::on_algoStartButton_clicked()
{

}

void MainWindow::on_algoComboBox_currentIndexChanged(int index)
{
    emit algoChanged(index);
//    qDebug() << index;
}

void MainWindow::on_algoActivationComboBox_currentIndexChanged(int index)
{
    emit algoActivationChanged(index);
}


void MainWindow::onGetResult(QString s)
{
    ui->resultText->setPlainText(s);
}

void MainWindow::on_resetButton_clicked()
{
    emit resetClicked(ui->configSpinBox->value());
}
