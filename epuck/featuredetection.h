#ifndef FEATUREDETECTION_H
#define FEATUREDETECTION_H

#include "camcapture.h"
#include "defines.h"
#include "imgproc.h"
#include <math.h>
#include <cvblob.h>
#include "tbb/parallel_for.h"
#include "tbb/blocked_range2d.h"
#include "commondefs.h"
#include "beliefstate.h"

//#define RENDER_BLOBS

class FeatureDetection
{
private:
    const int IMAGE_HEIGHT;
    const int IMAGE_WIDTH;
    int line_x1, line_x2, line_y1, line_y2;
    IplImage* seg_red;
    IplImage* seg_cyan;
    IplImage* seg_yellow;
    IplImage* seg_black;
    IplImage* seg_orange;
    IplImage* seg_purple;
    IplImage* seg_green;
    IplImage* seg_maroon;
    IplImage* labelImg;
//    void copyToVectorAndSort(std::vector<Blob> &v, cvb::CvBlobs &blobs, int sizeLimit = 12);
    void getPosition(cvb::CvBlobs &blobs, int botNumber);
    void printBot(CamCapture &cam, int num);
public:
    cvb::CvBlobs blobs_red;
    cvb::CvBlobs blobs_cyan;
    cvb::CvBlobs blobs_yellow;
    cvb::CvBlobs blobs_maroon;
    cvb::CvBlobs blobs_orange;
    cvb::CvBlobs blobs_purple;
    BeliefState bs;
    FeatureDetection(CamCapture &cam);
    void getBlobs(CamCapture &cam);
    void updateBeliefState(CamCapture &cam);
    inline double getDistance(CvPoint a, CvPoint b)
    {
        return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
    }
};

#endif
