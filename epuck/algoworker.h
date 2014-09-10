#ifndef ALGOWORKER_H
#define ALGOWORKER_H

#include <QThread>
#include <QObject>
#include "serial.h"
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <QMutex>
#include <QPixmap>
#include "camcapture.h"
#include "featuredetection.h"
#include "commondefs.h"
#include "hungarian.h"


//non simulation values
//#define ANGLE_THRESHOLD_COARSE 7.0*CV_PI/180.0
//#define DISTANCE_THRESHOLD_COARSE 100.0
//#define ANGLE_THRESHOLD_FINE 20.0*CV_PI/180.0
//#define DISTANCE_THRESHOLD_FINE 8.0
//#define FINAL_REACHED_THRESHOLD 20.0

//simulation values
#define ANGLE_THRESHOLD_COARSE 7.0*CV_PI/180.0
#define DISTANCE_THRESHOLD_COARSE 10.0
#define ANGLE_THRESHOLD_FINE 3.0*CV_PI/180.0
#define DISTANCE_THRESHOLD_FINE 8.0
#define FINAL_REACHED_THRESHOLD 18.0

enum AlgoState {SAVE_CURRENT_POSITION, MOVE_TO_SAVED_POSITION, MAKE_CIRCLE, MOVE_TO_CIRCLE, POSITIONING_ON_CIRCLE_1, POSITIONING_ON_CIRCLE_2, FINISHED, CALCULATE_POINTS_HUNGARIAN, MOVE_TO_POINTS};
enum Algorithm {MIDPOINT, NEW_ALGO, LYNDON, END_ALGO};
enum ActivationAlgorithm {PROBABILISTIC_0, PROBABILISTIC_1, PROBABILISTIC_HALF, DINING_PHILOSOPHER};
enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT, STOP};
class AlgoWorker : public QObject
{
    Q_OBJECT
public:
    AlgoWorker(QThread* _myThread, BeliefState **_bs, QMutex* _bsMutex);
public slots:
    void process();
    void onTimeout();
    void onStop();
    void onStopAlgo();
    void onAlgoChanged(int index);
    void onAlgoActivationChanged(int index);
signals:
    void gotLine(int x1, int y1, int x2, int y2);
    void printDestination(PointList p);
    void printCircle(Circle c);
    void finished();
    void error(QString err);
    void gotResult(QString s);


private:
    class LineSegment
    {
    public:
        double x1;
        double y1;
        double x2;
        double y2;
        LineSegment(double _x1, double _y1, double _x2, double _y2)
        {
            x1 = _x1;
            y1 = _y1;
            x2 = _x2;
            y2 = _y2;
        }
        LineSegment()
        {
            x1 = x2 = y1 = y2 = 0;
        }
    };


    HAL::Serial s[NUMBOTS];
    QTimer *timer;
    QMutex* bsMutex;
    QThread* myThread;
    BeliefState** bs;
    double getDistance(CvPoint a, CvPoint b);
    BeliefState localBS;
    AlgoState currentState;
    Algorithm currentAlgo;
    ActivationAlgorithm currentActivationAlgo;
    int line_x1, line_x2, line_y1, line_y2;
    void moveLeft(int n, int speed = 150);
    void moveRight(int n, int speed = 150);
    void moveForward(int n, int speed = 500);
    void moveBack(int n, int speed = 500);
    void moveStop(int n);
    void moveStopAll();
    void turnLedOn(int n);
    void turnLedOff(int n);
    CvPoint linePoint[2];
    CvPoint destinationPoint[NUMBOTS];
    CvPoint savedPosition[NUMBOTS];
    Circle destinationCircle;
    int robotActive;
    bool moveToPoint(CvPoint p, Bot bot, int n);
    bool moveToPointOpti(CvPoint p, Bot bot, int n);
    bool turnToPoint(CvPoint p, Bot bot, int n);
    int frameCounter;
    bool isRunning;
    void nextIteration();
    bool isLeftMost(int i);
    bool isRightMost(int i);
    bool isBotMoving[NUMBOTS];
    MoveDirection lastMove[NUMBOTS];
    int numBotMoving;
    int numActivations;
    int numRounds;
    double totDistance;
    void getLinePoints();
    CvPoint getPerpendicularPoint(int n);
    CvPoint getPerpendicularPoint(CvPoint p);
    CvPoint getMidPoint(CvPoint p1, CvPoint p2);
    int getCCWNeighbour(int n);
    int getCWNeighbour(int n);
    CvPoint getPerpPointOnCircle(int n);
    CvPoint getPerpPointOnCircle(CvPoint p, bool closer = true);
    CvPoint getPointToMoveAlgo1(int n);
    CvPoint getPerpPointOnCircleCloserToBot(CvPoint p, int n);
    CvPoint getMidOfNeighbours(int n);
    CvPoint getPointNewAlgo(int n);
    double pointToAngle(CvPoint p);
    double pointToAngle(int n);
    CvPoint angleToPoint(double angle);
    bool isAngleReachable(int n, double angle);
    bool isOneStableCCW(int n);
    bool isOneStableCW(int n);
    CvPoint getClosest(CvPoint p1, CvPoint p2, int n);
    void getNeighbourIndicesCircle(int n, int* indexList);
    Hungarian h;
    std::vector<LineSegment> ls;
    bool onLineSegment(CvPoint p, CvPoint q, CvPoint r);
    int orientation(CvPoint p, CvPoint q, CvPoint r);
    bool doIntersect(CvPoint p1, CvPoint q1, CvPoint p2, CvPoint q2);
    bool isClose(CvPoint p, CvPoint q);
    bool isIntersecting(LineSegment s1, LineSegment s2);
    CvPoint lineLineSegmentCircleIntersect(CvPoint centre, double radius, LineSegment s);
    CvPoint getDestinationVoronoi(int n);
};

#endif // ALGOWORKER_H
