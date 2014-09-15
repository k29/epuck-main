#include <QTime>
#include "algoworker.h"
#include <QDebug>
#include "QThread"
#include "fstream"
#include <algorithm>

using namespace std;

AlgoWorker::AlgoWorker(QThread* _myThread, BeliefState **_bs, QMutex* _bsMutex)
{
    myThread = _myThread;
    bs = _bs;
    bsMutex = _bsMutex;
    timer = new QTimer;
    connect(timer, SIGNAL(timeout()), this, SLOT(onTimeout()));
    ifstream f1;
    f1.open("../lineCalib.txt");
    f1>>line_x1;
    f1>>line_x2;
    f1>>line_y1;
    f1>>line_y2;
    f1.close();
}

double AlgoWorker::getDistance(CvPoint a, CvPoint b)
{
    return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}

void AlgoWorker::process()
{
    h.n = 3;
    int mat[3][3] = {
        {3, 3, 3},
        {3, 2, 3},
        {3, 2, 3}
    };
    for(int i = 0; i < 3; ++i)
    {
        for(int j = 0; j < 3; ++j)
        {
            h.cost[i][j] = mat[i][j];
        }
    }
    qDebug() << "Answer is " << h.hungarian();
    for(int i = 0; i < 3; ++i)
    {
        qDebug() << h.xy[i];
    }
    // allocate resources using new here
    QTime now = QTime::currentTime();
//    qsrand(50);
    qsrand(now.msec());
//    cout << "time " << now.msec();
    qDebug() << "Algo Worker thread started.";
    frameCounter = 0;
    isRunning = false;
    currentState = MOVE_TO_SAVED_POSITION;
    currentAlgo = NEW_ALGO;
    currentActivationAlgo = PROBABILISTIC_0;
    result.open("output5-dp.txt", ios::out);
    robotActive = 0;
    for(int i = 0; i < NUMBOTS; ++i)
    {
        lastMove[i] = STOP;
    }
#ifndef SIMULATION
    for(int i = 0; i < NUMBOTS; ++i)
    {
        char buffer[100];
        sprintf(buffer, "/dev/rfcomm%d", i);
        if(!s[i].Open(buffer, 9600))
        {
            qDebug() << "Couldn't open port "<< buffer;
        }
        else
        {
            qDebug() << "Port "<< buffer << " opened.";
        }

    }
#endif
#ifdef SIMULATION
    fstream f1;
    f1.open("positions10.txt", ios::out);
    for(int i = 0; i < 10; ++i)
    {
        for(int i = 0; i < NUMBOTS; ++i)
        {
            localBS.bot[i].isVisible = true;
            localBS.bot[i].x = qrand()%300+200;
            localBS.bot[i].y = qrand()%200 + 150;
            localBS.bot[i].angle = CV_PI;
            f1 << localBS.bot[i].x << " " << localBS.bot[i].y << endl;
        }
    }
    f1.close();



        fstream f2;
        f2.open("positions10.txt", ios::in);
        for(int j =0; j < 10; ++j)
        {
            for(int i = 0; i < NUMBOTS; ++i)
            {
                f2 >> savePositionNew[j][i].x;
                f2 >> savePositionNew[j][i].y;
            }
        }
        f2.close();
#endif
    globalCounter = 0;
    qDebug() << "Ok\n";
    timer->setSingleShot(true);
    timer->start(10);
}

void AlgoWorker::nextIteration()
{
    if(isRunning)
    {
        timer->setSingleShot(true);
        timer->start(10);
    }
}

bool AlgoWorker::turnToPoint(CvPoint p, Bot bot, int n)
{
    double angleTowardPoint = atan2(p.y - bot.y, p.x - bot.x) - bot.angle;
    while(angleTowardPoint > CV_PI)
        angleTowardPoint -= 2*CV_PI;
    while(angleTowardPoint < -CV_PI)
        angleTowardPoint += 2*CV_PI;

    if(angleTowardPoint > ANGLE_THRESHOLD_FINE)
    {
        moveRight(n);
    }
    else if(angleTowardPoint < -ANGLE_THRESHOLD_FINE)
    {
        moveLeft(n);
    }
    else
    {
        if(angleTowardPoint > ANGLE_THRESHOLD_COARSE)
        {
            moveRight(n);
//            moveRightSmall();
        }
        else if(angleTowardPoint < - ANGLE_THRESHOLD_COARSE)
        {
            moveLeft(n);
//            moveLeftSmall();
        }
        else
        {
            moveStop(n);
            return true;
        }
    }
    return false;
}


//add a turnToPoint
bool AlgoWorker::moveToPoint(CvPoint p, Bot &bot, int n)
{
#ifdef SIMULATION
    bot.x = p.x;
    bot.y = p.y;
    return true;
#endif
#ifndef SIMULATION
    double angleTowardPoint = atan2(p.y - bot.y, p.x - bot.x) - bot.angle;
    while(angleTowardPoint > CV_PI)
        angleTowardPoint -= 2*CV_PI;
    while(angleTowardPoint < -CV_PI)
        angleTowardPoint += 2*CV_PI;

//    qDebug() << "Angle towards: "<< angleTowardPoint;
    double distance = getDistance(p, cvPoint(bot.x, bot.y));
    // printf("Angle to point = %lf, Distance = %lf\n", angleTowardPoint*180./CV_PI, distance);

    if(angleTowardPoint > ANGLE_THRESHOLD_COARSE)
    {
        moveRight(n, 200);
    }
    else if(angleTowardPoint < -ANGLE_THRESHOLD_COARSE)
    {
        moveLeft(n, 200);
    }
    else
    {
        if(distance > DISTANCE_THRESHOLD_COARSE)
        {
            moveForward(n, 1000);
        }
        else
        {
            if(angleTowardPoint > ANGLE_THRESHOLD_FINE)
            {
                moveRight(n);
//                moveRightSmall();
            }
            else if(angleTowardPoint < - ANGLE_THRESHOLD_FINE)
            {
                moveLeft(n);
//                moveLeftSmall();
            }
            else
            {
                if(distance > DISTANCE_THRESHOLD_FINE)
                {
                    moveForward(n);
//                    moveForwardSmall();
                }
                else
                {
//                    qDebug() << "Bot reached.";
                    moveStop(n);
                    return true;
                }
            }
        }
    }
    return false;
#endif
}

//moves back or forward depending on angle
bool AlgoWorker::moveToPointOpti(CvPoint p, Bot &bot, int n)
{
#ifdef SIMULATION
    bot.x = p.x;
    bot.y = p.y;
    return true;
#endif
#ifndef SIMULATION
    double angleTowardPoint = atan2(p.y - bot.y, p.x - bot.x) - bot.angle;
    while(angleTowardPoint > CV_PI)
        angleTowardPoint -= 2*CV_PI;
    while(angleTowardPoint < -CV_PI)
        angleTowardPoint += 2*CV_PI;

//    qDebug() << "Angle towards: "<< angleTowardPoint;
    double distance = getDistance(p, cvPoint(bot.x, bot.y));
    // printf("Angle to point = %lf, Distance = %lf\n", angleTowardPoint*180./CV_PI, distance);


    if(fabs(angleTowardPoint) < CV_PI/2)
    {
        return moveToPoint(p, bot, n);
    }
    else
    {
        if(angleTowardPoint > 0)
            angleTowardPoint -= CV_PI;
        else
            angleTowardPoint += CV_PI;

        if(angleTowardPoint > ANGLE_THRESHOLD_COARSE)
        {
            moveRight(n, 200);
        }
        else if(angleTowardPoint < -ANGLE_THRESHOLD_COARSE)
        {
            moveLeft(n, 200);
        }
        else
        {
            if(distance > DISTANCE_THRESHOLD_COARSE)
            {
                moveBack(n, 1000);
            }
            else
            {
                if(angleTowardPoint > ANGLE_THRESHOLD_FINE)
                {
                    moveRight(n);
    //                moveRightSmall();
                }
                else if(angleTowardPoint < - ANGLE_THRESHOLD_FINE)
                {
                    moveLeft(n);
    //                moveLeftSmall();
                }
                else
                {
                    if(distance > DISTANCE_THRESHOLD_FINE)
                    {
                        moveBack(n);
    //                    moveForwardSmall();
                    }
                    else
                    {
//                        qDebug() << "Bot reached.";
                        moveStop(n);
                        return true;
                    }
                }
            }
        }
    }

    return false;
#endif
}


bool AlgoWorker::isLeftMost(int n)
{
    int smallestN = 0;
    double smallestX = 999999;
    for(int i = 0; i<NUMBOTS; ++i)
    {
        if(localBS.bot[i].x < smallestX)
        {
            smallestN = i;
            smallestX = localBS.bot[i].x;
        }
    }

    if(smallestN == n)
        return true;
    else
        return false;
}

bool AlgoWorker::isRightMost(int n)
{
    int largestN = 0;
    double largestX = -1;
    for(int i = 0; i<NUMBOTS; ++i)
    {
        if(localBS.bot[i].x > largestX)
        {
            largestN = i;
            largestX = localBS.bot[i].x;
        }
    }

    if(largestN == n)
        return true;
    else
        return false;
}


void AlgoWorker::getLinePoints()
{
    for(int i = 0; i<NUMBOTS; ++i)
    {
        if(isLeftMost(i))
        {
            qDebug() << "left" << i;
            linePoint[0].x = localBS.bot[i].x;
            linePoint[0].y = localBS.bot[i].y;
        }
        else if(isRightMost(i))
        {
            linePoint[1].x = localBS.bot[i].x;
            linePoint[1].y = localBS.bot[i].y;
            qDebug() << "right" << i;
        }
    }
}


CvPoint AlgoWorker::getPerpendicularPoint(CvPoint p3)
{
    CvPoint p1 = linePoint[0];
    CvPoint p2 = linePoint[1];
    CvPoint ans;
    ans.x = p3.x;
    ans.y = (p2.y - p1.y)*(p3.x - p1.x)/(p2.x -p1.x) + p1.y;
    return ans;
}

CvPoint AlgoWorker::getPerpendicularPoint(int n)
{
    CvPoint p3 = cvPoint(localBS.bot[n].x, localBS.bot[n].y);
    return getPerpendicularPoint(p3);
}

CvPoint AlgoWorker::getMidPoint(CvPoint p1, CvPoint p2)
{
    CvPoint ans;
    ans.x = (p1.x + p2.x)/2;
    ans.y = (p1.y + p2.y)/2;
    return ans;
}



CvPoint AlgoWorker::getPointToMoveAlgo1(int n)
{
    double refAngle;
    double angle[NUMBOTS];
    for(int i = 0; i < NUMBOTS; ++i)
    {
        angle[i] = atan2((double)localBS.bot[i].y - destinationCircle.centre.y,(double) localBS.bot[i].x - destinationCircle.centre.x);
        if(angle[i] < 0)
        {
            angle[i] += 2*CV_PI;
        }
    }
    refAngle = angle[n];
    cv::Point2f p[2];
    int m[2];
    m[0] = m[1] = 0;
    double minDiff = 10000.0, maxDiff = -10000.0;
    for(int i = 0; i < NUMBOTS; ++i)
    {
        double tempAngle = refAngle - angle[i];
        while(tempAngle < 0)
            tempAngle += 2*CV_PI;

        if(tempAngle < minDiff && i!=n)
        {
            m[0] = i;
            minDiff = tempAngle;
        }
        if(tempAngle > maxDiff && i!=n)
        {
            m[1] = i;
            maxDiff = tempAngle;
        }
    }
    p[0] = cv::Point2f(localBS.bot[m[0]].x, localBS.bot[m[0]].y);
    p[1] = cv::Point2f(localBS.bot[m[1]].x, localBS.bot[m[1]].y);

    //p1 and p2 are neighbours of bot
    //get midpoint and getperppointoncircle



    cv::Point2f a[2];
    a[0].x = destinationCircle.centre.x + destinationCircle.radius*cos((angle[m[0]] + angle[m[1]])/2);
    a[0].y = destinationCircle.centre.y + destinationCircle.radius*sin((angle[m[0]] + angle[m[1]])/2);

    a[1].x = destinationCircle.centre.x + destinationCircle.radius*cos((angle[m[0]] + angle[m[1]])/2 + CV_PI);
    a[1].y = destinationCircle.centre.y + destinationCircle.radius*sin((angle[m[0]] + angle[m[1]])/2 + CV_PI);


    //p1 and p2 are neighbours of bot
    //get midpoint and getperppointoncircle

//    cv::Point2f mid = getMidPoint(p[0], p[1]);
//    cv::Point2f a[2];
    double angleFromCentre[2];
//    a[0] = getPerpPointOnCircle(mid, true);
//    a[1] = getPerpPointOnCircle(mid, false);
//    //a1 and a2 are 2 points to choose from.
//    //choose the one which makes acute angle from centre to bot
    double botAngle = atan2(localBS.bot[n].y - destinationCircle.centre.y, localBS.bot[n].x - destinationCircle.centre.x);
    for(int i = 0; i < 2; ++i)
    {
        angleFromCentre[i] = atan2(a[i].y - destinationCircle.centre.y, a[i].x - destinationCircle.centre.x);
        angleFromCentre[i] -= botAngle;
        while(angleFromCentre[i] < -CV_PI)
            angleFromCentre[i] += 2*CV_PI;
        while(angleFromCentre[i] > CV_PI)
            angleFromCentre[i] -= 2*CV_PI;
    }

    cv::Point2f midOfNeighbours;
    if(fabs(angleFromCentre[0]) < CV_PI/2)
    {
        midOfNeighbours = a[0];
    }
    else
    {
        midOfNeighbours = a[1];
    }

    // now get mid of this with bot, and get
    // perp point closer to circle

    cv::Point2f mid2 = getMidPoint(cvPoint(localBS.bot[n].x, localBS.bot[n].y), midOfNeighbours);

//    return mid2;
    return getPerpPointOnCircle(mid2);
}


CvPoint AlgoWorker::getMidOfNeighbours(int n)
{
    double refAngle;
    double angle[NUMBOTS];
    for(int i = 0; i < NUMBOTS; ++i)
    {
        angle[i] = atan2((double)localBS.bot[i].y - destinationCircle.centre.y, (double)localBS.bot[i].x - destinationCircle.centre.x);
        if(angle[i] < 0)
        {
            angle[i] += 2*CV_PI;
        }
    }
    refAngle = angle[n];
    cv::Point2f p[2];
    int m[2];
    m[0] = m[1] = 0;
    double minDiff = 10000.0, maxDiff = -10000.0;
    for(int i = 0; i < NUMBOTS; ++i)
    {
        double tempAngle = refAngle - angle[i];
        while(tempAngle < 0)
            tempAngle += 2*CV_PI;

        if(tempAngle < minDiff && i!=n)
        {
            m[0] = i;
            minDiff = tempAngle;
        }
        if(tempAngle > maxDiff && i!=n)
        {
            m[1] = i;
            maxDiff = tempAngle;
        }
    }
    p[0] = cvPoint(localBS.bot[m[0]].x, localBS.bot[m[0]].y);
    p[1] = cvPoint(localBS.bot[m[1]].x, localBS.bot[m[1]].y);



    cv::Point2f a[2];
    a[0].x = destinationCircle.centre.x + destinationCircle.radius*cos((angle[m[0]] + angle[m[1]])/2);
    a[0].y = destinationCircle.centre.y + destinationCircle.radius*sin((angle[m[0]] + angle[m[1]])/2);

    a[1].x = destinationCircle.centre.x + destinationCircle.radius*cos((angle[m[0]] + angle[m[1]])/2 + CV_PI);
    a[1].y = destinationCircle.centre.y + destinationCircle.radius*sin((angle[m[0]] + angle[m[1]])/2 + CV_PI);


    //p1 and p2 are neighbours of bot
    //get midpoint and getperppointoncircle

//    cv::Point2f mid = getMidPoint(p[0], p[1]);
//    cv::Point2f a[2];
    double angleFromCentre[2];
//    a[0] = getPerpPointOnCircle(mid, true);
//    a[1] = getPerpPointOnCircle(mid, false);
//    //a1 and a2 are 2 points to choose from.
//    //choose the one which makes acute angle from centre to bot
    double botAngle = atan2(localBS.bot[n].y - destinationCircle.centre.y, localBS.bot[n].x - destinationCircle.centre.x);
    for(int i = 0; i < 2; ++i)
    {
        angleFromCentre[i] = atan2(a[i].y - destinationCircle.centre.y, a[i].x - destinationCircle.centre.x);
        angleFromCentre[i] -= botAngle;
        while(angleFromCentre[i] < -CV_PI)
            angleFromCentre[i] += 2*CV_PI;
        while(angleFromCentre[i] > CV_PI)
            angleFromCentre[i] -= 2*CV_PI;
    }

    cv::Point2f midOfNeighbours;
    if(fabs(angleFromCentre[0]) < CV_PI/2)
    {
        midOfNeighbours = a[0];
    }
    else
    {
        midOfNeighbours = a[1];
    }

    return midOfNeighbours;
}


CvPoint AlgoWorker::getPerpPointOnCircle(CvPoint p, bool closer)
{
    cv::Point2f p1, p2;
    cv::Point2f centre = destinationCircle.centre;
    double radius = destinationCircle.radius;
//    qDebug() << p.x << p.y;
    if(p.x != centre.x)
    {
        double slope = ((double)p.y - (double)centre.y)/((double)p.x - (double)centre.x);
        p1.x = (double)centre.x + radius*sqrt(1.0/(slope*slope + 1));
        p1.y = (double)centre.y + slope*(p1.x - centre.x);
        p2.x = (double)centre.x - radius*sqrt(1.0/(slope*slope + 1));
        p2.y = (double)centre.y + slope*(p2.x - centre.x);
    }
    else
    {
        p1.x = centre.x;
        p1.y = centre.y + radius;
        p2.x = centre.x;
        p2.y = centre.y - radius;
    }

//    qDebug() << "Point: X:" << p2.x << "Y: " << p2.y;
//    qDebug() << "Point: X:" << p1.x << "Y: " << p1.y;
//    qDebug() << "Centre: " << centre.x << centre.y;
//    qDebug() << "Radius:" << radius;
    if(closer)
    {
        if(getDistance(p1, p) < getDistance(p2, p))
            return p1;
        else
            return p2;
    }
    else
    {
        if(getDistance(p1, p) > getDistance(p2, p))
            return p1;
        else
            return p2;
    }
}

CvPoint AlgoWorker::getPerpPointOnCircleCloserToBot(CvPoint p, int n)
{
    CvPoint p1, p2;
    cv::Point2f centre = destinationCircle.centre;
    double radius = destinationCircle.radius;
    if(p.x != centre.x)
    {
        double slope = ((double)p.y - (double)centre.y)/((double)p.x - (double)centre.x);
        p1.x = centre.x + radius*sqrt(1.0/(slope*slope + 1));
        p1.y = centre.y + slope*(p1.x - centre.x);
        p2.x = centre.x - radius*sqrt(1.0/(slope*slope + 1));
        p2.y = centre.y + slope*(p2.x - centre.x);
    }
    else
    {
        p1.x = centre.x;
        p1.y = centre.y +radius;
        p2.x = centre.x;
        p2.y = centre.y - radius;
    }
    CvPoint botLocation = cvPoint(localBS.bot[n].x, localBS.bot[n].y);
    if(getDistance(p1, botLocation) < getDistance(p2, botLocation))
        return p1;
    else
        return p2;
}

CvPoint AlgoWorker::getPerpPointOnCircle(int n)
{
    return getPerpPointOnCircle(cvPoint(localBS.bot[n].x, localBS.bot[n].y));
}

void AlgoWorker::getNeighbourIndicesCircle(int n, int* indexList)
{
    double refAngle;
    double angle[NUMBOTS];
    for(int i = 0; i < NUMBOTS; ++i)
    {
        angle[i] = atan2(localBS.bot[i].y - destinationCircle.centre.y, localBS.bot[i].x - destinationCircle.centre.x);
        if(angle[i] < 0)
        {
            angle[i] += 2*CV_PI;
        }
    }
    refAngle = angle[n];
    CvPoint p[2];
    int m[2];
    m[0] = m[1] = 0;
    double minDiff = 10000.0, maxDiff = -10000.0;
    for(int i = 0; i < NUMBOTS; ++i)
    {
        double tempAngle = refAngle - angle[i];
        while(tempAngle < 0)
            tempAngle += 2*CV_PI;

        if(tempAngle < minDiff && i!=n)
        {
            m[0] = i;
            minDiff = tempAngle;
        }
        if(tempAngle > maxDiff && i!=n)
        {
            m[1] = i;
            maxDiff = tempAngle;
        }
    }
    indexList[0] = m[0];
    indexList[1] = m[1];
    return;
}


//int AlgoWorker::getLeftNeighbourIndex(int n)
//{
//    return 0;
//}

int AlgoWorker::getCWNeighbour(int n)
{
    int idList[2];
    getNeighbourIndicesCircle(n, idList);
    return idList[1];
}

int AlgoWorker::getCCWNeighbour(int n)
{
    int idList[2];
    getNeighbourIndicesCircle(n, idList);
    return idList[0];
}

bool AlgoWorker::isAngleReachable(int n, double angle)
{
    double angleCCW = pointToAngle(getCCWNeighbour(n));
    double angleCW = pointToAngle(getCWNeighbour(n));

    if(angleCCW < angleCW) //normal case, not passing through 0
    {
        if(angle > angleCCW && angle < angleCW)
            return true;
        else
            return false;
    }
    else
    {
        //passing through 0
        if(angle > angleCCW || angle < angleCW)
            return true;
        else
            return false;
    }
}


bool AlgoWorker::isOneStableCCW(int n)
{
    double angle1 = 2*pointToAngle(getCCWNeighbour(n)) - pointToAngle(getCCWNeighbour(getCCWNeighbour(n)));
    while(angle1 < 0)
        angle1 += 2*CV_PI;
    while(angle1 > 2*CV_PI)
        angle1 -= 2*CV_PI;
    CvPoint p1 = angleToPoint(angle1);
    if(getDistance(p1, cvPoint(localBS.bot[n].x, localBS.bot[n].y)) < 20)
        return true;
    else
        return false;
}

bool AlgoWorker::isOneStableCW(int n)
{
    double angle1 = 2*pointToAngle(getCWNeighbour(n)) - pointToAngle(getCWNeighbour(getCWNeighbour(n)));
    while(angle1 < 0)
        angle1 += 2*CV_PI;
    while(angle1 > 2*CV_PI)
        angle1 -= 2*CV_PI;
    CvPoint p1 = angleToPoint(angle1);
    if(getDistance(p1, cvPoint(localBS.bot[n].x, localBS.bot[n].y)) < 20)
        return true;
    else
        return false;
}

CvPoint AlgoWorker::getClosest(CvPoint p1, CvPoint p2, int n)
{
    CvPoint p = cvPoint(localBS.bot[n].x, localBS.bot[n].y);
    if(getDistance(p, p1) < getDistance(p, p2))
        return p1;
    else
        return p2;
}

CvPoint AlgoWorker::getPointNewAlgo(int n, bool closer)
{
/**************
If robot is in no stable state
  - It checks if it can make two stable configuration with its neighbors.
    - If YES, then it does that
  - If the robot can't make two stable configuration then it sees that if it can
    make one stable configuration with its neighbors
     - If NO then the robot stays silent and relinquishes the chance to its neighbors
     - If YES then the robot has to decide if it should make one stable config. with its
       clockwise or anti-clockwise neighbor
       - Three cases
         (a) both the neighbors are one stable
             - then make one stable config. with the one which is at minimum angular distance
         (b) both no stable
             - then make one stable config. with the one which is at minimum angular distance
         (c) one of them is one stable
             - then make one stable config. with this robot so that it becomes two stable
************/

    //angle increases clockwise!
    //check if 2-stable possible
    const double stableAngle = 2.0*CV_PI/((double)NUMBOTS);
    double angle = pointToAngle(getCWNeighbour(n)) - pointToAngle(getCCWNeighbour(n));
    while(angle < 0)
        angle += 2*CV_PI;
    while(angle > 2*CV_PI)
        angle -= 2*CV_PI;

    if(fabs(angle/2.0 - stableAngle) < ANGLE_THRESHOLD_FINE)
    {
        //2-stable possible
//        qDebug() << "2 stable possible!";
        return getMidOfNeighbours(n);
    }

    //check if 1-stable possible

    if(angle < stableAngle)
    {
//        qDebug() << "1-stable not possible!";
        return cvPoint(localBS.bot[n].x, localBS.bot[n].y);
    }

    //1-stable is possible
    //check if any neighbour is 1-stable
    //if either is, move to its corresponding point,
    //else move to CCW one

    double angle1 = pointToAngle(getCCWNeighbour(n)) - pointToAngle(getCCWNeighbour(getCCWNeighbour(n)));
    while(angle1 < 0)
        angle1 += 2*CV_PI;
    while(angle1 > 2*CV_PI)
        angle1 -= 2*CV_PI;

    double angle2 = pointToAngle(getCWNeighbour(getCWNeighbour(n))) - pointToAngle(getCWNeighbour(n));
    while(angle2 < 0)
        angle2 += 2*CV_PI;
    while(angle2 > 2*CV_PI)
        angle2 -= 2*CV_PI;

    if((fabs(angle1 - stableAngle) < ANGLE_THRESHOLD_FINE && fabs(angle2 - stableAngle) < ANGLE_THRESHOLD_FINE) ||
       (fabs(angle1 - stableAngle) > ANGLE_THRESHOLD_FINE && fabs(angle2 - stableAngle) > ANGLE_THRESHOLD_FINE) )
    {
        //choose closer one
        CvPoint p1 = angleToPoint(pointToAngle(getCCWNeighbour(n)) + stableAngle);
        CvPoint p2 = angleToPoint(pointToAngle(getCWNeighbour(n)) - stableAngle);
        CvPoint bot = cvPoint(localBS.bot[n].x, localBS.bot[n].y);
        if(getDistance(bot, p1) < getDistance(bot, p2))
        {
            if(closer)
                return p1;
            else
                return p2;
        }
        else
        {
            if(closer)
                return p2;
            else
                return p1;
        }
    }
    else
    {

        if(fabs(angle1 - stableAngle) < ANGLE_THRESHOLD_FINE)
        {
            return angleToPoint(pointToAngle(getCCWNeighbour(n)) + stableAngle);
        }
        else
        {
            return angleToPoint(pointToAngle(getCWNeighbour(n)) - stableAngle);
        }
    }
}


CvPoint AlgoWorker::angleToPoint(double angle)
{
    CvPoint p;
    p.x = destinationCircle.centre.x + cos(angle)*destinationCircle.radius;
    p.y = destinationCircle.centre.y + sin(angle)*destinationCircle.radius;
    return p;
}

double AlgoWorker::pointToAngle(CvPoint p)
{
    double angle = atan2(p.y - destinationCircle.centre.y, p.x - destinationCircle.centre.x);
    if(angle < 0)
    {
        angle += 2*CV_PI;
    }
    return angle;
}

double AlgoWorker::pointToAngle(int n)
{
    return pointToAngle(cvPoint(localBS.bot[n].x, localBS.bot[n].y));
}

bool AlgoWorker::onLineSegment(CvPoint p, CvPoint q, CvPoint r)
{
    if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
        q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
       return true;

    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int AlgoWorker::orientation(CvPoint p, CvPoint q, CvPoint r)
{
    // See 10th slides from following link for derivation of the formula
    // http://www.dcs.gla.ac.uk/~pat/52233/slides/Geometry1x1.pdf
    int val = (q.y - p.y) * (r.x - q.x) -
              (q.x - p.x) * (r.y - q.y);

    if (val == 0) return 0;  // colinear

    return (val > 0)? 1: 2; // clock or counterclock wise
}

// The main function that returns true if line LineSegment 'p1q1'
// and 'p2q2' intersect.
bool AlgoWorker::doIntersect(CvPoint p1, CvPoint q1, CvPoint p2, CvPoint q2)
{
    // Find the four orientations needed for general and
    // special cases
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on LineSegment p1q1
    if (o1 == 0 && onLineSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are colinear and q2 lies on LineSegment p1q1
    if (o2 == 0 && onLineSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on LineSegment p2q2
    if (o3 == 0 && onLineSegment(p2, p1, q2)) return true;

     // p2, q2 and q1 are colinear and q1 lies on LineSegment p2q2
    if (o4 == 0 && onLineSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}



bool AlgoWorker::isClose(CvPoint p, CvPoint q)
{
    if(fabs(p.x - q.x) < 4 && fabs(p.y - q.y) < 4)
        return true;
    else
        return false;
}


bool AlgoWorker::isIntersecting(LineSegment s1, LineSegment s2)
{
    CvPoint p1, p2, q1, q2;
    p1.x = s1.x1;
    p1.y = s1.y1;
    p2.x = s1.x2;
    p2.y = s1.y2;

    q1.x = s2.x1;
    q1.y = s2.y1;
    q2.x = s2.x2;
    q2.y = s2.y2;
    if(isClose(p1, q1) || isClose(p1, q2) || isClose(p2, q1) ||isClose(p2, q2))
        return false;
    if(doIntersect(p1, p2, q1, q2))
        return true;
    else
        return false;
}


CvPoint AlgoWorker::lineLineSegmentCircleIntersect(CvPoint centre, double radius, LineSegment s)
{
    double a = s.x1 - s.x2;
    double b = s.x2 - centre.x;
    double c = s.y1 - s.y2;
    double d = s.y2 - centre.y;
    double discriminantSquare = (2.*a*b + 2.*c*d)*(2.*a*b + 2.*c*d) - 4.*(a*a + c*c)*(b*b + d*d - radius*radius);
    if(discriminantSquare < 0)
    {
        CvPoint p;
        p.x = -999;
        p.y = -999;
        return p;
    }
    double t1 = ((-(2.*a*b + 2.*c*d))+ sqrt(discriminantSquare))/(2.*(a*a + c*c));
    double t2 = ((-(2.*a*b + 2.*c*d))- sqrt(discriminantSquare))/(2.*(a*a + c*c));
    double t;
    if(t1 >= 0 && t1 <= 1)
    {
        t = t1;
    }
    else if(t2 >=0 && t2 <= 1)
    {
        t = t2;
    }
    else
    {
        CvPoint p;
        p.x = -999;
        p.y = -999;
        return p;
    }

    CvPoint p;
    // t = t2;
//    cout << t << endl;
    p.x = t*s.x1 + (1.0 - t)*s.x2;
    p.y = t*s.y1 + (1.0 - t)*s.y2;
    return p;
}



CvPoint AlgoWorker::getDestinationVoronoi(int n)
{
    int numIntersections = 0;
    if(fabs(getDistance(cvPoint(localBS.bot[n].x, localBS.bot[n].y), destinationCircle.centre) - destinationCircle.radius) < 10)
        return cvPoint(localBS.bot[n].x, localBS.bot[n].y);
    CvPoint finalPoints[2];
    for (int i = 0; i < ls.size() && numIntersections < 2; ++i)
    {
        LineSegment temp1 = LineSegment(localBS.bot[n].x, localBS.bot[n].y, ls[i].x1, ls[i].y1);
        LineSegment temp2 = LineSegment(localBS.bot[n].x, localBS.bot[n].y, ls[i].x2, ls[i].y2);
        bool isPartOfCell = true;
        for (int j = 0; j < ls.size(); ++j)
        {
            // if(j == i) continue;
            if(isIntersecting(temp1, ls[j]) || isIntersecting(temp2, ls[j]))
            {
                isPartOfCell = false;
                break;
            }
        }
        if(isPartOfCell)
        {
//            cvLine(img, cvPoint(ls[i].x1, ls[i].y1), cvPoint(ls[i].x2, ls[i].y2), cvScalar(255, 255, 0), 3);
            CvPoint p = lineLineSegmentCircleIntersect(destinationCircle.centre, destinationCircle.radius, ls[i]);
            if(p.x != -999 && p.y != -999)
            {
                finalPoints[numIntersections] = p;
                numIntersections++;
                if(numIntersections == 2)
                {
                    if(getDistance(finalPoints[0], finalPoints[1]) < 3)
                    {
                        numIntersections = 1;
                    }
                }

//                cvLine(img, cvPoint(ls[i].x1, ls[i].y1), cvPoint(ls[i].x2, ls[i].y2), cvScalar(127, 127, 0), 3);
//                cvCircle(img, p, 3, cvScalar(127, 0, 0), 3);
            }
        }
//        cvShowImage("asd", img);
        // cvWaitKey();
    }
    if(numIntersections == 2)
    {
        qDebug() << "2 intersections for bot " << n ;
//        return finalPoints[1];
        CvPoint midp = getMidPoint(finalPoints[0], finalPoints[1]);
        return getPerpPointOnCircle(midp);
    }
    else
    {
        //0 or 1 intersections, means go to closest point
        qDebug() << "NO INTERSECTION!!! " << n;
        CvPoint ans;
        double minDistance = 9999999.0;
        for(int i = 0; i < ls.size(); ++i)
        {
            LineSegment temp1 = LineSegment(localBS.bot[n].x, localBS.bot[n].y, ls[i].x1, ls[i].y1);
            LineSegment temp2 = LineSegment(localBS.bot[n].x, localBS.bot[n].y, ls[i].x2, ls[i].y2);
            bool isPartOfCell = true;
            for (int j = 0; j < ls.size(); ++j)
            {
                // if(j == i) continue;
                if(isIntersecting(temp1, ls[j]) || isIntersecting(temp2, ls[j]))
                {
                    isPartOfCell = false;
                    break;
                }
            }
            if(isPartOfCell)
            {
                CvPoint p1 = cvPoint(ls[i].x1, ls[i].y1);
                CvPoint p2 = cvPoint(ls[i].x2, ls[i].y2);
                double distance = getDistance(p1, destinationCircle.centre);
                if(fabs(distance - destinationCircle.radius) < minDistance)
                {
                    minDistance = fabs(distance - destinationCircle.radius);
                    ans = p1;
                }
                distance = getDistance(p2, destinationCircle.centre);
                if(fabs(distance - destinationCircle.radius) < minDistance)
                {
                    minDistance = fabs(distance - destinationCircle.radius);
                    ans = p2;
                }
           }
        }
//        qDebug () << ans.x << " " << ans.y;
//        qDebug () << localBS.bot[n].x <<" "<< localBS.bot[n].y;
        return ans;
    }
}

void AlgoWorker::onTimeout()
{
    bool isBSAvailable = false;
    bsMutex->lock();
    if(bs)
    {
        if(*bs)
        {
#ifdef SIMULATION
            *(*bs) = localBS;
#endif
#ifndef SIMULATION
            localBS = *(*bs);
#endif
            isBSAvailable = true;
//             qDebug() << "Printing bs value " << localBS.bot[0].angle;
        }
    }
    bsMutex->unlock();
//    localBS.bot[0].angle += 2;
//    printf("Outside lock\n");

//    timer->setSingleShot(true);
//    timer->start(10);

//    return;
    int allBotVisible = 0;

    for(int i = 0; i < NUMBOTS; ++i)
    {
        if(localBS.bot[i].isVisible)
            allBotVisible++;
    }
//    if(isBSAvailable)
//     qDebug() << "Y222OOOOOOOOOOO";
//    qDebug() << avail3;
    if(allBotVisible == NUMBOTS && isBSAvailable && isRunning)
    {
//        qDebug() << "YOOOOOOOOOOO\n";

        switch(currentState)
        {

        case SAVE_CURRENT_POSITION:
            totDistance = 0.0;
            numRounds = 0;
            numActivations = 0;
            for(int i = 0; i < NUMBOTS; ++i)
            {
                savedPosition[i] = cvPoint(localBS.bot[i].x, localBS.bot[i].y);
            }
            currentState = MAKE_CIRCLE;
            break;

        case MOVE_TO_SAVED_POSITION:
        {
            int numCompleted = 0;
            totDistance = 0.0;
            numRounds = 0;
            numActivations = 0;
            for(int i = 0; i < NUMBOTS; ++i)
            {
                if(moveToPoint(savePositionNew[globalCounter][i], localBS.bot[i], i))
                    numCompleted++;
            }
            if(numCompleted == NUMBOTS)
            {
                qDebug() << "Reached random points";
                currentState = MAKE_CIRCLE;
            }
        }
            break;
        case MAKE_CIRCLE:
        {
//            qDebug() << "in make circle";

            std::vector<cv::Point> pList;
            cv::Point2f centre;
            float radius;
            for(int i=0; i < NUMBOTS; ++i)
            {
                pList.push_back(cv::Point(localBS.bot[i].x, localBS.bot[i].y));
            }
            cv::minEnclosingCircle(pList, centre, radius);
            destinationCircle.centre = centre;
//            qDebug() << centre.x << centre.y;
            destinationCircle.radius = radius;
            emit printCircle(destinationCircle);
            ls.clear();
            fstream input;
            input.open("in.txt", ios::out);
            for(int i = 0; i < NUMBOTS; ++i)
            {
                input << localBS.bot[i].x << " " << localBS.bot[i].y << endl;
            }
            input.close();
            system("./voronoi < in.txt > out.txt");

            fstream output;
            output.open("out.txt", ios::in);
            double x1, x2, y1, y2;
            while(output >> x1 >> y1 >> x2 >> y2)
            {
                ls.push_back(LineSegment(x1, y1, x2, y2));
            }
            output.close();
            for(int i = 0; i < NUMBOTS; ++i)
            {
                destinationPoint[i] = getDestinationVoronoi(i);
                if(destinationPoint[i].y > 470)
                {
                    destinationPoint[i].y = 470;
                }
                if(destinationPoint[i].y < 10)
                {
//                    qDebug() << "a";
                    destinationPoint[i].y = 10;
                }
                totDistance += getDistance(destinationPoint[i], cvPoint(localBS.bot[i].x, localBS.bot[i].y));
//                qDebug() << "destinationPoint[" << i << "] = cvPoint(" << destinationPoint[i].x << "," << destinationPoint[i].y << ");";
//                qDebug() << destinationPoint[i].x << destinationPoint[i].y;
            }

            PointList temp;
            for(int i = 0; i < NUMBOTS; ++i)
            {
                temp.p[i] = destinationPoint[i];
            }
//            usleep(2000000);
            emit printDestination(temp);
//            isRunning = false;
            //            sleep(10);
//            sleep(10);
//            usleep(2000000);
            if(currentAlgo == LYNDON)
                currentState = CALCULATE_POINTS_HUNGARIAN;
            else
                currentState = MOVE_TO_CIRCLE;
            for(int i = 0; i<NUMBOTS; ++i)
            {
                turnLedOff(i);
            }
            qDebug() << "Moving to circle started";
        }

        break;

        case MOVE_TO_CIRCLE:
        {
            int numCompleted = 0;
            for(int i = 0; i < NUMBOTS; ++i)
            {
                if(moveToPoint(destinationPoint[i], localBS.bot[i], i))
                    numCompleted++;
            }
            if(numCompleted == NUMBOTS)
            {
                bool allOnCircle = true;
                for(int i = 0; i < NUMBOTS; ++i)
                {
                    if(fabs(getDistance(destinationPoint[i], destinationCircle.centre) - destinationCircle.radius) > FINAL_REACHED_THRESHOLD)
                    {
                        allOnCircle = false;
                        break;
                    }
                }
                if(allOnCircle == false)
                {
                    qDebug() << "All not on circle";
                    currentState = MAKE_CIRCLE;
                }
                else
                {
                    qDebug() << "Positioning on circle started";
                    currentState = POSITIONING_ON_CIRCLE_1;
    //                sleep(10);

                    numRounds = 0;
                    numActivations = 0;
                    robotActive = 0;
                }
            }
        }
        break;

        case POSITIONING_ON_CIRCLE_1:
        {
//            qDebug() << "New Iteration Start.";
            //check if robots are evenly distributed
            bool isFinalPositionReached = true;
            const double stableAngle = 2.0*CV_PI/((double)NUMBOTS);
            for(int i = 0; i < NUMBOTS; ++i)
            {
//                CvPoint dest = getMidOfNeighbours(i);
                double angleFormed = pointToAngle(getCWNeighbour(i)) - pointToAngle(i);
                while(angleFormed < 0)
                    angleFormed += 2*CV_PI;
                while(angleFormed > 2*CV_PI)
                    angleFormed -= 2*CV_PI;
                if(fabs(angleFormed - stableAngle) > 10.0*CV_PI/180.0)
//                if(getDistance(dest, cvPoint(localBS.bot[i].x, localBS.bot[i].y)) > FINAL_REACHED_THRESHOLD)
                {
                    isFinalPositionReached = false;
                    break;
                }
            }
            if(isFinalPositionReached)
            {
                for(int i = 0; i < NUMBOTS; ++i)
                {
                    turnLedOff(i);
                }
                currentState = FINISHED;
                break;
            }
            if(currentActivationAlgo != DINING_PHILOSOPHER)
            {
                for(int i = 0; i < NUMBOTS; i++)
                {
                    if(currentActivationAlgo == PROBABILISTIC_HALF)
                    {
                        if(qrand()%2 == 0)
                        {
                            isBotMoving[i] = true;
                            if(currentAlgo == NEW_ALGO)
                                destinationPoint[i] = getPointNewAlgo(i);
                            else
                                destinationPoint[i] = getPointToMoveAlgo1(i);
                            qDebug() << "Bot" << i << "moving";
                        }
                        else
                            isBotMoving[i] = false;
                    }
                    else if(currentActivationAlgo == PROBABILISTIC_1)
                    {
                        if(currentAlgo == NEW_ALGO)
                            destinationPoint[i] = getPointNewAlgo(i);
                        else
                            destinationPoint[i] = getPointToMoveAlgo1(i);
                        isBotMoving[i] = true;
                    }
                    else if(currentActivationAlgo == PROBABILISTIC_QUARTER)
                    {
                        if(qrand()%4 == 0)
                        {
                            isBotMoving[i] = true;
                            if(currentAlgo == NEW_ALGO)
                                destinationPoint[i] = getPointNewAlgo(i);
                            else
                                destinationPoint[i] = getPointToMoveAlgo1(i);
                            qDebug() << "Bot" << i << "moving";
                        }
                        else
                            isBotMoving[i] = false;
                    }
                    else
                        isBotMoving[i] = false;

                }
                if(currentAlgo == NEW_ALGO)
                {
                    destinationPoint[robotActive] = getPointNewAlgo(robotActive);
                    int numStuckCount = 0;
                    for(int i = 0; i < NUMBOTS; ++i)
                    {
                        if(getDistance(cvPoint(localBS.bot[i].x, localBS.bot[i].y), getPointNewAlgo(i)) < 10)
                        {
                            numStuckCount++;
                        }
                    }
                    if(numStuckCount == NUMBOTS)
                    {
                        qDebug() << "REACHED DEADLOCK";
//                        std::vector<int> canMove;
                        double largestAngle = 0;
                        int robotLargestAngle;
                        for(int i = 0; i < NUMBOTS; ++i)
                        {
                            double angleFormed = pointToAngle(getCWNeighbour(i)) - pointToAngle(i);
                            while(angleFormed < 0)
                                angleFormed += 2*CV_PI;
                            while(angleFormed > 2*CV_PI)
                                angleFormed -= 2*CV_PI;
                            if(angleFormed > largestAngle)
                            {
                                largestAngle = angleFormed;
                                robotLargestAngle = i;
                            }
                        }
//                        qDebug() << canMove.size() << " can move";
//                        if(canMove.size() == 2)
//                        {
//                            if(pointToAngle(canMove[0]) > pointToAngle(canMove[1]))
//                            {
//                                destinationPoint[canMove[0]] = getPointNewAlgo(canMove[0], false);
//                                isBotMoving[canMove[0]] = true;
//                            }
//                            else
//                            {
//                                destinationPoint[canMove[1]] = getPointNewAlgo(canMove[1], false);
//                                isBotMoving[canMove[1]] = true;
//                            }
//                        }
                        destinationPoint[robotLargestAngle] = getPointNewAlgo(robotLargestAngle, false);
                        isBotMoving[robotLargestAngle] = true;
//                        sleep(2);
//                        sleep(100);
                    }
                }
                else
                    destinationPoint[robotActive] = getPointToMoveAlgo1(robotActive);
                isBotMoving[robotActive] = true;
            }
            else if(currentActivationAlgo == DINING_PHILOSOPHER)
            {
                int state[NUMBOTS];
                for(int i = 0; i < NUMBOTS; ++i)
                {
                    state[i] = -1;
                }

                state[robotActive] = 1;
                isBotMoving[robotActive] = true;
                if(currentAlgo == NEW_ALGO)
                    destinationPoint[robotActive] = getPointNewAlgo(robotActive);
                else if(currentAlgo == MIDPOINT)
                    destinationPoint[robotActive] = getPointToMoveAlgo1(robotActive);

                for(int i = 0; i < NUMBOTS; ++i)
                {
                    int indexList[2];
                    getNeighbourIndicesCircle(i, indexList);
                    if(state[i] == -1)
                    {
                        if(state[indexList[0]] == 1 || state[indexList[1]] == 1)
                        {
                            state[i] = 0;
                            isBotMoving[i] = false;
                        }
                        else
                        {
                            state[i] = 1;
                            isBotMoving[i] = true;
                            if(currentAlgo == NEW_ALGO)
                                destinationPoint[i] = getPointNewAlgo(i);
                            else if(currentAlgo == MIDPOINT)
                                destinationPoint[i] = getPointToMoveAlgo1(i);
                        }
                    }
                }
            }

            numBotMoving = 0;
            numRounds++;
            for(int i = 0; i<NUMBOTS; ++i)
            {
                if(isBotMoving[i])
                {
                    turnLedOn(i);
                    numActivations++;
                    numBotMoving++;
                    totDistance += getDistance(cvPoint(localBS.bot[i].x, localBS.bot[i].y), destinationPoint[i]);
                }
                else
                {
                    turnLedOff(i);
                }
            }
            //for showing
            PointList temp;
            for(int i = 0; i < NUMBOTS; ++i)
            {
                temp.p[i] = destinationPoint[i];
            }
            emit printDestination(temp);
            currentState = POSITIONING_ON_CIRCLE_2;
//            qDebug() << "In Positioning 1";
        }
            break;

        case POSITIONING_ON_CIRCLE_2:
        {
//            qDebug() << "In Positioning 2";
            int count = 0;
            for(int i = 0; i < NUMBOTS; ++i)
            {
                if(isBotMoving[i])
                {
                    if(moveToPointOpti(destinationPoint[i], localBS.bot[i], i))
                    {
                        count++;
                    }
                }
            }
            if(currentAlgo == LYNDON)
            {
                if(count == NUMBOTS)
                {
                    currentState = FINISHED;
                }
            }
            else if(count == numBotMoving)
            {
//                qDebug() << "inside";
                robotActive++;
                if(robotActive >= NUMBOTS)
                    robotActive = 0;
                currentState = POSITIONING_ON_CIRCLE_1;
            }

        }
        break;

        case CALCULATE_POINTS_HUNGARIAN:
        {
            h.n = NUMBOTS;
            //calculate the destination points
            //assume leader is destinationPoint[0]
            const double stableAngle = 2.0*CV_PI/((double)NUMBOTS);
            CvPoint myPoint[NUMBOTS];
            for(int i = 0; i < NUMBOTS; ++i)
            {
                myPoint[i] = angleToPoint(pointToAngle(destinationPoint[0]) + stableAngle*i);
            }

            PointList temp;
            for(int i = 0; i < NUMBOTS; ++i)
            {
                temp.p[i] = myPoint[i];
            }
//            usleep(2000000);
            emit printDestination(temp);

            //fill cost matrix
            for(int i = 0; i < NUMBOTS; ++i)
            {
                for(int j = 0; j < NUMBOTS; ++j)
                {
                    h.cost[i][j] = getDistance(cvPoint(localBS.bot[i].x, localBS.bot[i].y), myPoint[j]);
                }
            }
            totDistance = h.hungarian();
            for(int i = 0; i < NUMBOTS; ++i)
            {
                destinationPoint[i] = myPoint[h.xy[i]];
                isBotMoving[i] = true;
            }
            currentState = POSITIONING_ON_CIRCLE_2;
        }

        break;

        case MOVE_TO_POINTS:

        break;
        case FINISHED:
            QString s;
            s = QString("Number of activations: ") + QString::number(numActivations) + "\n";
            s += "Number of rounds: " + QString::number(numRounds) + "\n";
            s += "Total distance: " + QString::number((int)totDistance) + "\n";
            qDebug() << s;
            emit gotResult(s);


            result << s.toStdString();
            result << endl << endl;
//            result.close();
//            qDebug() << "Finished";
//             qDebug() << "Number of activations: " << numActivations;
//             qDebug() << "Number of rounds: " << numRounds;
//             qDebug() << "Total Distance: " << totDistance;
//             sleep(5);
            moveStopAll();
            currentState = MOVE_TO_SAVED_POSITION;
            isRunning = true;
            globalCounter++;
            sleep(2);
            if(globalCounter == 10)
            {
                isRunning = false;
            }
//            currentAlgo = (Algorithm) (((int)currentAlgo + 1)%((int)END_ALGO));
            break;

        }
    }
    else
    {
//        qDebug() << "Stopping all";

        moveStopAll();
    }

    frameCounter++;
//    qDebug() << frameCounter;
//    usleep(100000);
//    qDebug()<< "starting next iteration";
    timer->setSingleShot(true);
    timer->start(10);
//    nextIteration();
}

void AlgoWorker::onStop()
{
//    s.WriteByte('t');
//    s.Close();

    moveStopAll();
    for(int i = 0; i < NUMBOTS; ++i)
    {
//        s[i].WriteString(std::string("d,0,0"));
        s[i].Close();
    }

//    usleep(1000000);

//    for(int i = 0; i < NUMBOTS; ++i)
//    {
//        s[i].Close();
//    }
    qDebug() << "AlgoWorker stopping.";
    myThread->exit();
}


void AlgoWorker::moveForward(int n, int speed)
{
//    cout<<"yoman"<<endl;
#ifdef SIMULATION
    CvPoint p1 = cvPoint(localBS.bot[n].x, localBS.bot[n].y);
    CvPoint p2;
    p2.x = p1.x + 5.0*cos(localBS.bot[n].angle);
    p2.y = p1.y + 5.0*sin(localBS.bot[n].angle);
    if(p2.x < 0)
        p2.x = 0;
    if(p2.y < 0)
        p2.y = 0;
    if(p2.x > 640)
        p2.x = 640;
    if(p2.y > 480)
        p2.y = 480;
    localBS.bot[n].x = p2.x;
    localBS.bot[n].y = p2.y;
#endif
#ifndef SIMULATION
    if(lastMove[n] == FORWARD)
        return;
    char buffer[200];
    sprintf(buffer, "d,%d,%d", speed, speed);
    s[n].WriteString(std::string(buffer));
    lastMove[n] = FORWARD;
#endif
}

void AlgoWorker::moveBack(int n, int speed)
{
#ifdef SIMULATION
    CvPoint p1 = cvPoint(localBS.bot[n].x, localBS.bot[n].y);
    CvPoint p2;
    p2.x = p1.x - 5.0*cos(localBS.bot[n].angle);
    p2.y = p1.y - 5.0*sin(localBS.bot[n].angle);
    if(p2.x < 0)
        p2.x = 0;
    if(p2.y < 0)
        p2.y = 0;
    if(p2.x > 640)
        p2.x = 640;
    if(p2.y > 480)
        p2.y = 480;
    localBS.bot[n].x = p2.x;
    localBS.bot[n].y = p2.y;
#endif
#ifndef SIMULATION
    if(lastMove[n] == BACKWARD)
        return;
    char buffer[200];
    speed = - speed;
    sprintf(buffer, "d,%d,%d", speed, speed);
    s[n].WriteString(std::string(buffer));
    lastMove[n] = BACKWARD;
#endif
}

void AlgoWorker::moveLeft(int n, int speed)
{
#ifdef SIMULATION
    localBS.bot[n].angle -= CV_PI*2.0/180.0;
    while(localBS.bot[n].angle > 2*CV_PI)
        localBS.bot[n].angle -= 2*CV_PI;
    while(localBS.bot[n].angle < 0)
        localBS.bot[n].angle += 2*CV_PI;
#endif
#ifndef SIMULATION
    if(lastMove[n] == LEFT)
        return;
    char buffer[200];
    sprintf(buffer, "d,-%d,%d", speed, speed);
    s[n].WriteString(std::string(buffer));
    lastMove[n] = LEFT;
#endif
}

void AlgoWorker::moveRight(int n, int speed)
{
#ifdef SIMULATION

    localBS.bot[n].angle += CV_PI*2.0/180.0;
    while(localBS.bot[n].angle > 2*CV_PI)
        localBS.bot[n].angle -= 2*CV_PI;
    while(localBS.bot[n].angle < 0)
        localBS.bot[n].angle += 2*CV_PI;

#endif
#ifndef SIMULATION
    if(lastMove[n] == RIGHT)
        return;
    char buffer[200];
    sprintf(buffer, "d,%d,-%d", speed, speed);
    s[n].WriteString(std::string(buffer));
    lastMove[n] = RIGHT;
#endif
}

void AlgoWorker::moveStop(int n)
{
    if(lastMove[n] == STOP)
        return;
    s[n].WriteString(std::string("d,0,0"));
    lastMove[n] = STOP;
}

void AlgoWorker::moveStopAll()
{
    for(int i = 0; i < NUMBOTS; ++i)
    {
        moveStop(i);
    }
}


void AlgoWorker::onStopAlgo()
{
    qDebug() << "Start/stop algorithm";
    if(isRunning == true)
        isRunning = false;
    else
        isRunning = true;

    //add code for stopping all robots
    moveStopAll();
}

void AlgoWorker::onAlgoChanged(int index)
{
    currentAlgo = (Algorithm) index;
}

void AlgoWorker::onAlgoActivationChanged(int index)
{
//    qDebug() << "changed";
    currentActivationAlgo = (ActivationAlgorithm) index;
}

void AlgoWorker::turnLedOn(int n)
{
#ifndef SIMULATION
    s[n].WriteString(std::string("b,1"));
#endif
}

void AlgoWorker::turnLedOff(int n)
{
#ifndef SIMULATION
    s[n].WriteString(std::string("b,0"));
#endif
}
