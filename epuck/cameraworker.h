#ifndef CAMERAWORKER_H
#define CAMERAWORKER_H

#include <QThread>
#include <QObject>
#include "serial.h"
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <QMutex>
#include <QPixmap>
#include "camcapture.h"
#include "featuredetection.h"
#include "beliefstate.h"

class CameraWorker : public QObject
{
    Q_OBJECT

public:
    CameraWorker(QThread* _myThread, QMutex *_myMutex, BeliefState **_bs, QMutex* _bsMutex);

public slots:
    void process();
//    void moveForward();
//    void moveLeft();
//    void moveRight();
//    void moveStop();
    void onTimeout();
    void onStop();
    void onGotLine(int x1, int y1, int x2, int y2);
    void onPrintDestination(PointList5 p);
    void onPrintCircle(Circle c);

signals:
    void finished();
    void error(QString err);
    void imageReady(QPixmap *pm);

private:
    HAL::Serial s;
    QTimer *timer;
    IplImage* frame;
    QMutex* myMutex;
    QMutex* bsMutex;
    QPixmap *myPixmap;
    CvCapture* capture;
    CamCapture* cc;
    FeatureDetection* fd;
    QThread* myThread;
    BeliefState** bs;
    bool isLinePresent;
    CvPoint linePoint[2];
    CvPoint destinationPoint[5];
    Circle circle;
    bool isDestinationPresent;
    bool isCirclePresent;
};

#endif // CAMERAWORKER_H
