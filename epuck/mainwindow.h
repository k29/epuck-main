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
    void onGetResult(QString s);

    void on_algoComboBox_currentIndexChanged(int index);

    void on_algoActivationComboBox_currentIndexChanged(int index);

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
    void stopAlgo();
    void algoChanged(int index);
    void algoActivationChanged(int index);
};

#endif // MAINWINDOW_H
