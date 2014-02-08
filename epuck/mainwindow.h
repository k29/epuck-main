#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv2/opencv.hpp>
#include <QMutex>
#include "cameraworker.h"
#include "beliefstate.h"
#include "algoworker.h"
#include "QTime"


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
    void onCamImageReady(QPixmap *pm);

    void on_algoStartButton_clicked();

    void on_algoStopButton_clicked();


private:
    Ui::MainWindow *ui;
    void IplImage2QImage(IplImage *iplImg, QImage *image);
    QMutex* camMutex;
    QMutex* bsMutex;
    QThread* cameraThread;
    QThread* algoThread;
    CameraWorker* cw;
    AlgoWorker* aw;
    BeliefState* bs;
    QTime getfps;

signals:
    void stopCamThread();
    void stopAlgoThread();
    void startAlgo();
    void stopAlgo();
};

#endif // MAINWINDOW_H
