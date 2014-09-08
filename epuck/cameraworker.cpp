#include "cameraworker.h"
#include <QDebug>
#include <string>
#include "commondefs.h"
#include "defines.h"
#include "fstream"
#include "QThread"
#define CAMNUM 1

using namespace std;

CameraWorker::CameraWorker(QThread* _myThread, QMutex* _myMutex, BeliefState** _bs, QMutex* _bsMutex)
{
    myThread = _myThread;
    myMutex = _myMutex;
    bs = _bs;
    bsMutex = _bsMutex;
    timer = new QTimer;
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    myPixmap = NULL;
}

void CameraWorker::process()
{
    // allocate resources using new here
    qDebug() << "Camera Worker thread started.";
#ifndef SIMULATION
    cc = new CamCapture(true);//, "../pic-2.jpg");
#endif
#ifdef SIMULATION
    cc = new CamCapture(false, "./test.png");
#endif
    cc->init(CAMNUM);
    fd = new FeatureDetection(*cc);
#ifdef SIMULATION
    fd->initBSSimulation();
    bsMutex->lock();
    if(*bs)
        delete (*bs);
    (*bs) = new BeliefState(fd->bs);
    bsMutex->unlock();
#endif
    timer->setSingleShot(true);
    timer->start(10);
    isLinePresent = false;
    isDestinationPresent = false;
}



void CameraWorker::onTimeout()
{
    cc->getImage();
#ifndef SIMULATION
    fd->updateBeliefState(*cc);
#endif

    bsMutex->lock();
#ifndef SIMULATION
    if(*bs)
        delete (*bs);
    (*bs) = new BeliefState(fd->bs);
#endif
#ifdef SIMULATION
//    fd->updateBeliefStateSimulation();
    //also draw bots on the image
    fd->printBotSimulation(cc, **bs);
#endif
    bsMutex->unlock();

    if(isLinePresent)
    {
        cvLine(cc->rgbimg,linePoint[0], linePoint[1], cvScalar(0,255,0), 3);
    }
    if(isDestinationPresent)
    {
        for(int i = 0; i < 5; ++i)
        {
            cvCircle(cc->rgbimg, destinationPoint[i], 3, cvScalar(0,0,0), 3);
            cvCircle(cc->rgbimg, destinationPoint[i], 1, cvScalar(255,255,255), 3);
        }
    }
    if(isCirclePresent)
    {
//        qDebug() << "circle present";
//        qDebug() << circle.centre.x << circle.centre.y;
        cvCircle(cc->rgbimg, cvPoint(circle.centre.x, circle.centre.y), circle.radius, cvScalar(0,0,255), 3);
    }
//    qDebug() << "Bot state:" << fd->bot.x<< fd->bot.y<< fd->bot.angle;
#ifdef CAMCAPTURE_DEBUG
    frame = cc->showSeg;
#endif
#ifndef CAMCAPTURE_DEBUG
    frame = cc->rgbimg;
#endif
    cvCvtColor(frame, frame,CV_BGR2RGB);
    QImage qimg((uchar*)frame->imageData, frame->width, frame->height, frame->widthStep, QImage::Format_RGB888);
    myMutex->lock();
    if(myPixmap)
        delete myPixmap;
    myPixmap = new QPixmap(QPixmap::fromImage(qimg));
    myMutex->unlock();
    emit imageReady(myPixmap);

    timer->setSingleShot(true);
    timer->start(10);
}


void CameraWorker::onStop()
{
    if(cc)
    {
        delete cc;
    }

    myMutex->lock();
    if(myPixmap)
        delete myPixmap;
    myMutex->unlock();
    qDebug() << "CameraWorker stopping.";
    myThread->exit();
}

void CameraWorker::onGotLine(int x1, int y1, int x2, int y2)
{
    qDebug() << "Got line in cameraworker";
    linePoint[0] = cvPoint(x1, y1);
    linePoint[1] = cvPoint(x2, y2);
    isLinePresent = true;
}

void CameraWorker::onPrintDestination(PointList p)
{
    isDestinationPresent = true;
    for(int i = 0; i < 5; ++i)
    {
        destinationPoint[i] = p.p[i];
    }
}


void CameraWorker::onPrintCircle(Circle c)
{
    isCirclePresent = true;
    circle = c;
}
