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
    // allocate resources using new here
    QTime now = QTime::currentTime();
    qsrand(now.msec());
    qDebug() << "Algo Worker thread started.";
    frameCounter = 0;
    isRunning = false;
    currentState = SAVE_CURRENT_POSITION;
    currentAlgo = DINING_PHILOSOPHER;
    robotActive = 0;
    for(int i = 0; i < 5; ++i)
    {
        lastMove[i] = STOP;
    }
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
bool AlgoWorker::moveToPoint(CvPoint p, Bot bot, int n)
{
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
}

//moves back or forward depending on angle
bool AlgoWorker::moveToPointOpti(CvPoint p, Bot bot, int n)
{
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
}


bool AlgoWorker::isLeftMost(int n)
{
    int smallestN = 0;
    double smallestX = 999999;
    for(int i = 0; i<5; ++i)
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
    for(int i = 0; i<5; ++i)
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
    for(int i = 0; i<5; ++i)
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

CvPoint AlgoWorker::getLeftNeighbourPoint(int n)
{
    double refX = localBS.bot[n].x;
    int botNum = 0;
    double x = -9999;
    for(int i = 0; i < 5; ++i)
    {
        if(localBS.bot[i].x > x && localBS.bot[i].x < refX && i!=n)
        {
            botNum = i;
            x = localBS.bot[i].x;
        }
    }
    CvPoint ans = cvPoint(localBS.bot[botNum].x, localBS.bot[botNum].y);
    return ans;
}

CvPoint AlgoWorker::getRightNeighbourPoint(int n)
{
    double refX = localBS.bot[n].x;
    int botNum = 0;
    double x = 9999;
    for(int i = 0; i < 5; ++i)
    {
        if(localBS.bot[i].x < x && localBS.bot[i].x > refX && i!=n)
        {
            botNum = i;
            x = localBS.bot[i].x;
        }
    }
    CvPoint ans = cvPoint(localBS.bot[botNum].x, localBS.bot[botNum].y);
    return ans;
}


CvPoint AlgoWorker::getPointToMoveAlgo1(int n)
{
    double refAngle;
    double angle[5];
    for(int i = 0; i < 5; ++i)
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
    for(int i = 0; i < 5; ++i)
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


CvPoint AlgoWorker::getPointToMoveAlgo2(int n)
{
    double refAngle;
    double angle[5];
    for(int i = 0; i < 5; ++i)
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
    for(int i = 0; i < 5; ++i)
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
    double angle[5];
    for(int i = 0; i < 5; ++i)
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
    for(int i = 0; i < 5; ++i)
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

void AlgoWorker::onTimeout()
{
    bool isBSAvailable = false;
//    printf(":here\n");
    bsMutex->lock();
    if(bs)
    {
        if(*bs)
        {
            localBS = *(*bs);
            isBSAvailable = true;
//             qDebug() << "Printing bs value " << localBS.bot[0].angle;
        }
    }
    bsMutex->unlock();
//    printf("Outside lock\n");

//    timer->setSingleShot(true);
//    timer->start(10);

//    return;
    int allBotVisible = 0;

    for(int i = 0; i < 5; ++i)
    {
        if(localBS.bot[i].isVisible)
            allBotVisible++;
    }
//    qDebug() << "Y222OOOOOOOOOOO";

//    qDebug() << avail3;
    if(allBotVisible == 5 && isBSAvailable)
    {
//        qDebug() << "YOOOOOOOOOOO\n";
        switch(currentState)
        {

        case SAVE_CURRENT_POSITION:
            for(int i = 0; i < 5; ++i)
            {
                savedPosition[i] = cvPoint(localBS.bot[i].x, localBS.bot[i].y);
            }
            currentState = MAKE_CIRCLE;
            break;

        case MOVE_TO_SAVED_POSITION:
        {
            int numCompleted = 0;
            for(int i = 0; i < 5; ++i)
            {
                if(moveToPoint(savedPosition[i], localBS.bot[i], i))
                    numCompleted++;
            }
            if(numCompleted == 5)
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
            for(int i=0; i < 5; ++i)
            {
                pList.push_back(cv::Point(localBS.bot[i].x, localBS.bot[i].y));
            }
            cv::minEnclosingCircle(pList, centre, radius);
            destinationCircle.centre = centre;
//            qDebug() << centre.x << centre.y;
            destinationCircle.radius = radius;
            emit printCircle(destinationCircle);

            for(int i = 0; i < 5; ++i)
            {
                destinationPoint[i] = getPerpPointOnCircle(i);
                if(destinationPoint[i].y > 470)
                {
                    destinationPoint[i].y = 470;
                }
                if(destinationPoint[i].y < 10)
                {
//                    qDebug() << "a";
                    destinationPoint[i].y = 10;
                }
//                qDebug() << "destinationPoint[" << i << "] = cvPoint(" << destinationPoint[i].x << "," << destinationPoint[i].y << ");";
//                qDebug() << destinationPoint[i].x << destinationPoint[i].y;
            }

            PointList5 temp;
            for(int i = 0; i < 5; ++i)
            {
                temp.p[i] = destinationPoint[i];
            }
//            usleep(2000000);
            emit printDestination(temp);
//            sleep(10);
//            usleep(2000000);
            currentState = MOVE_TO_CIRCLE;
            for(int i = 0; i<5; ++i)
            {
                turnLedOff(i);
            }
            qDebug() << "Moving to circle started";
        }

        break;

        case MOVE_TO_CIRCLE:
        {
            int numCompleted = 0;
            for(int i = 0; i < 5; ++i)
            {
                if(moveToPoint(destinationPoint[i], localBS.bot[i], i))
                    numCompleted++;
            }
            if(numCompleted == 5)
            {
                qDebug() << "Positioning on circle started";
                currentState = POSITIONING_ON_CIRCLE_1;
                sleep(10);
                numRounds = 0;
                numActivations = 0;
                totDistance = 0.0;
                robotActive = 0;
            }
        }
        break;

        case POSITIONING_ON_CIRCLE_1:
        {
//            qDebug() << "New Iteration Start.";
            //check if robots are evenly distributed
            bool isFinalPositionReached = true;
            for(int i = 0; i < 5; ++i)
            {
                CvPoint dest = getPointToMoveAlgo2(i);
                if(getDistance(dest, cvPoint(localBS.bot[i].x, localBS.bot[i].y)) > 20)
                {
                    isFinalPositionReached = false;
                    break;
                }
            }
            if(isFinalPositionReached)
            {
                for(int i = 0; i < 5; ++i)
                {
                    turnLedOff(i);
                }
                currentState = FINISHED;
                break;
            }
            if(currentAlgo != DINING_PHILOSOPHER)
            {
                for(int i = 0; i < 5; i++)
                {
                    if(currentAlgo == PROBABILISTIC_HALF)
                    {
                        if(qrand()%2 == 0)
                        {
                            isBotMoving[i] = true;
                            destinationPoint[i] = getPointToMoveAlgo1(i);
                            qDebug() << "Bot" << i << "moving";
                        }
                        else
                            isBotMoving[i] = false;
                    }
                    else if(currentAlgo == PROBABILISTIC_1)
                    {
                        destinationPoint[i] = getPointToMoveAlgo1(i);
                        isBotMoving[i] = true;
                    }
                    else
                        isBotMoving[i] = false;

                }
                destinationPoint[robotActive] = getPointToMoveAlgo1(robotActive);
                isBotMoving[robotActive] = true;
            }
            else if(currentAlgo == DINING_PHILOSOPHER)
            {
                int state[5];
                for(int i = 0; i < 5; ++i)
                {
                    state[i] = -1;
                }

                //-1 = state not set, 1 = moving, 0 = not moving

                state[robotActive] = 1;
                isBotMoving[robotActive] = true;
                destinationPoint[robotActive] = getPointToMoveAlgo2(robotActive);

                for(int i = 0; i < 5; ++i)
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
                            destinationPoint[i] = getPointToMoveAlgo2(i);
                        }
                    }
                }
            }

            numBotMoving = 0;
            numRounds++;
            for(int i = 0; i<5; ++i)
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
            PointList5 temp;
            for(int i = 0; i < 5; ++i)
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
            for(int i = 0; i < 5; ++i)
            {
                if(isBotMoving[i])
                {
                    if(moveToPointOpti(destinationPoint[i], localBS.bot[i], i))
                    {
                        count++;
                    }
                }
            }
            if(count == numBotMoving)
            {
//                qDebug() << "inside";
                robotActive++;
                if(robotActive >= 5)
                    robotActive = 0;
                currentState = POSITIONING_ON_CIRCLE_1;
            }

        }
        break;

        case FINISHED:
//            qDebug() << "Finished";
             qDebug() << "Number of activations: " << numActivations;
             qDebug() << "Number of rounds: " << numRounds;
             qDebug() << "Total Distance: " << totDistance;
             sleep(5);
            moveStopAll();
            currentState = MOVE_TO_SAVED_POSITION;
            currentAlgo = (Algorithm) ((int)currentAlgo + 1);
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
    for(int i = 0; i < 5; ++i)
    {
//        s[i].WriteString(std::string("d,0,0"));
        s[i].Close();
    }

//    usleep(1000000);

//    for(int i = 0; i < 5; ++i)
//    {
//        s[i].Close();
//    }
    qDebug() << "AlgoWorker stopping.";
    myThread->exit();
}

void AlgoWorker::moveForward(int n, int speed)
{
    if(lastMove[n] == FORWARD)
        return;
    char buffer[200];
    sprintf(buffer, "d,%d,%d", speed, speed);
    s[n].WriteString(std::string(buffer));
    lastMove[n] = FORWARD;
}

void AlgoWorker::moveBack(int n, int speed)
{
    if(lastMove[n] == BACKWARD)
        return;
    char buffer[200];
    speed = - speed;
    sprintf(buffer, "d,%d,%d", speed, speed);
    s[n].WriteString(std::string(buffer));
    lastMove[n] = BACKWARD;
}

void AlgoWorker::moveLeft(int n, int speed)
{
    if(lastMove[n] == LEFT)
        return;
    char buffer[200];
    sprintf(buffer, "d,-%d,%d", speed, speed);
    s[n].WriteString(std::string(buffer));
    lastMove[n] = LEFT;
}

void AlgoWorker::moveRight(int n, int speed)
{
    if(lastMove[n] == RIGHT)
        return;
    char buffer[200];
    sprintf(buffer, "d,%d,-%d", speed, speed);
    s[n].WriteString(std::string(buffer));
    lastMove[n] = RIGHT;
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
    for(int i = 0; i < 5; ++i)
    {
        moveStop(i);
    }
}


void AlgoWorker::onStartAlgo()
{
    qDebug() << "Starting/Resuming algorithm";
    isRunning = true;
    nextIteration();
}

void AlgoWorker::onStopAlgo()
{
    qDebug() << "Pausing algorithm";
    isRunning = false;

    //add code for stopping all robots
    moveStopAll();
}

void AlgoWorker::turnLedOn(int n)
{
    s[n].WriteString(std::string("b,1"));
}

void AlgoWorker::turnLedOff(int n)
{
    s[n].WriteString(std::string("b,0"));
}
