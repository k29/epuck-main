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

void CameraWorker::printNodes()
{
    int width = 640/NODE_COLS;
    int height = 480/NODE_ROWS;
    for(int i = 0; i <= 640; i+= width)
        cvLine(cc->rgbimg, cvPoint(i, 0), cvPoint(i, 480), cvScalar(0,0,0), 1);
    for(int j = 0; j <= 480; j+= height)
        cvLine(cc->rgbimg, cvPoint(0, j), cvPoint(640, j), cvScalar(0,0,0), 1);

    for(int i = 0; i < NODE_ROWS; ++i)
    {
        for(int j = 0; j < NODE_COLS; ++j)
        {
            if((*bs)->node[i][j] < 0)
            {
                cvRectangle(cc->rgbimg, cvPoint(i*width, j*height), cvPoint(i*width+width, j*height+height),
                            cvScalar(0,0,0), -1);
            }
            else
            {
                char buf[20];
                sprintf(buf, "%d", (*bs)->node[i][j]);
                CvFont font;
                cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1);
                cvPutText(cc->rgbimg, buf, cvPoint(i*width +width/2, j*height +height/2), &font, cvScalar(0,0,0));
//                cvPutText(cc->rgbimg, buf, cvPoint(p1.x-1, p1.y - 21), &font, cvScalar(0,255,255));
            }
        }
    }
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
    printNodes();
#endif
    bsMutex->unlock();

//    fstream input;
//    input.open("in.txt", ios::out);
//    for(int i = 0; i < NUMBOTS; ++i)
//    {
//        input << (*(*bs)).bot[i].x << " " << (*(*bs)).bot[i].y << endl;
//    }
//    input.close();
//    system("./voronoi < in.txt > out.txt");

//    fstream output;
//    output.open("out.txt", ios::in);
//    double x1, x2, y1, y2;
//    while(output >> x1 >> y1 >> x2 >> y2)
//    {
//        cvLine(cc->rgbimg, cvPoint(x1, y1), cvPoint(x2, y2), cvScalar(255, 0, 0), 1);
//    }


    if(isLinePresent)
    {
        cvLine(cc->rgbimg,linePoint[0], linePoint[1], cvScalar(0,255,0), 3);
    }
    if(isDestinationPresent)
    {
        for(int i = 0; i < NUMBOTS; ++i)
        {
            cvCircle(cc->rgbimg, destinationPoint[i], 3, cvScalar(0,0,0), 3);
            cvCircle(cc->rgbimg, destinationPoint[i], 1, cvScalar(255,255,255), 3);
        }
    }
    if(isCirclePresent)
    {
//        qDebug() << "circle present";
//        qDebug() << circle.centre.x << circle.centre.y;
        cvCircle(cc->rgbimg, cvPoint(circle.center.x, circle.center.y), circle.radius, cvScalar(0,0,255), 3);
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
    for(int i = 0; i < NUMBOTS; ++i)
    {
        destinationPoint[i] = p.p[i];
    }
}


void CameraWorker::onPrintCircle(Circle c)
{
    isCirclePresent = true;
    circle = c;
}
