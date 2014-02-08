#include "featuredetection.h"
#include <fstream>
#include <QDebug>

using namespace cvb;
using namespace tbb;
using namespace std;
FeatureDetection::FeatureDetection(CamCapture &cam): IMAGE_HEIGHT(cam.height()), IMAGE_WIDTH(cam.width())
{
    ifstream f1;
    f1.open("../lineCalib.txt");
    f1>>line_x1;
    f1>>line_x2;
    f1>>line_y1;
    f1>>line_y2;
    f1.close();
    seg_red = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), 8, 1);
    seg_cyan = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), 8, 1);
    seg_yellow = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), 8, 1);
    seg_black = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), 8, 1);
    seg_orange = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), 8, 1);
    seg_purple = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), 8, 1);
    seg_maroon = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), 8, 1);
    seg_green = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), 8, 1);
    labelImg = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_LABEL, 1);
}



//For parallel segmenting
class SegmentImages_all {
    IplImage* my_seg_yellow;
    IplImage* my_seg_cyan;
    IplImage* my_seg_red;
    IplImage* my_seg_orange;
    IplImage* my_seg_purple;
    IplImage* my_seg_black;
    CamCapture* my_cam;
public:
    void operator()( const blocked_range2d<size_t>& r ) const {
        for( size_t x=r.rows().begin(); x!=r.rows().end(); ++x )
        {
            for( size_t y=r.cols().begin(); y!=r.cols().end(); ++y )
            {
                if(my_cam->isC1(x, y))
                    returnPixel1C(my_seg_orange, x, y) = 255;

                if(my_cam->isC2(x, y))
                    returnPixel1C(my_seg_purple, x, y) = 255;

                if(my_cam->isYellow(x, y))
                    returnPixel1C(my_seg_yellow, x, y) = 255;

                if(my_cam->isCyan(x, y))
                    returnPixel1C(my_seg_cyan, x, y) = 255;

                if(my_cam->isRed(x, y))
                    returnPixel1C(my_seg_red, x, y) = 255;

                if(my_cam->isBlack(x, y))
                    returnPixel1C(my_seg_black, x, y) = 255;
            }
        }
    }
    SegmentImages_all(IplImage* &seg_yellow, IplImage* &seg_cyan, IplImage* &seg_red, IplImage* &seg_orange, IplImage* &seg_purple, IplImage* &seg_black, CamCapture &cam) :
        my_seg_yellow(seg_yellow), my_seg_cyan(seg_cyan), my_seg_red(seg_red), my_seg_orange(seg_orange), my_seg_black(seg_black),  my_seg_purple(seg_purple), my_cam(&cam)
    {}
};



void FeatureDetection::getBlobs(CamCapture &cam)
{
    cvZero(seg_yellow);
    cvZero(seg_cyan);
    cvZero(seg_red);
    cvZero(seg_orange);
    cvZero(seg_purple);
    cvZero(seg_black);
    parallel_for( blocked_range2d<size_t>(line_x1, line_x2, 16, line_y1, line_y2, 32),
                  SegmentImages_all(seg_yellow,seg_cyan,seg_red,seg_orange, seg_purple, seg_black, cam) );
    // IplConvKernel *morphkernel = cvCreateStructuringElementEx(3,3,0,0,CV_SHAPE_RECT);
    // cvMorphologyEx(seg_yellow, seg_yellow, NULL, morphkernel, CV_MOP_OPEN, 1);
    cvLabel(seg_yellow, labelImg, blobs_yellow);
    cvFilterByArea(blobs_yellow, BLOB_MIN_AREA, 1000000);
#ifdef RENDER_BLOBS
    cvRenderBlobs(labelImg, blobs_yellow, cam.rgbimg, cam.rgbimg);
#endif
    cvLabel(seg_cyan, labelImg, blobs_cyan);
    cvFilterByArea(blobs_cyan, BLOB_MIN_AREA, 1000000);
#ifdef RENDER_BLOBS
    cvRenderBlobs(labelImg, blobs_cyan, cam.rgbimg, cam.rgbimg);
#endif
    cvLabel(seg_red, labelImg, blobs_red);
    cvFilterByArea(blobs_red, BLOB_MIN_AREA, 1000000);
#ifdef RENDER_BLOBS
    cvRenderBlobs(labelImg, blobs_red, cam.rgbimg, cam.rgbimg);
#endif
    cvLabel(seg_orange, labelImg, blobs_orange);
    cvFilterByArea(blobs_orange, BLOB_MIN_AREA, 1000000);
#ifdef RENDER_BLOBS
    cvRenderBlobs(labelImg, blobs_orange, cam.rgbimg, cam.rgbimg);
#endif
    cvLabel(seg_purple, labelImg, blobs_purple);
    cvFilterByArea(blobs_purple, BLOB_MIN_AREA, 1000000);
#ifdef RENDER_BLOBS
    cvRenderBlobs(labelImg, blobs_purple, cam.rgbimg, cam.rgbimg);
#endif

    // int i = 0;

    // i= 0;
    // for (CvBlobs::const_iterator it=blobs_black.begin(); it!=blobs_black.end(); ++it, i++)
    // {
    //   std::cout << "Black #" << i << ": Area=" << it->second->area << ", Centroid=(" << it->second->centroid.x << ", " << it->second->centroid.y << ")" << std::endl;
    // }

    // i= 0;
    // for (CvBlobs::const_iterator it=blobs_yellow.begin(); it!=blobs_yellow.end(); ++it, i++)
    // {
    //   std::cout << "Yellow #" << i << ": Area=" << it->second->area << ", Centroid=(" << it->second->centroid.x << ", " << it->second->centroid.y << ")" << std::endl;
    // }
}





void FeatureDetection::getPosition(cvb::CvBlobs &blobs, int botNumber)
{
    CvBlobs::const_iterator c;
    int nBlobs = 0;
    int area = 0;
    //get largest blob
    for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it)
    {
        nBlobs++;
        if(it->second->area > area)
        {
            area = it->second->area;
            c = it;
        }
    }

    bs.bot[botNumber].isVisible = false;

    if(nBlobs)
    {
        CvBlob* mainBlob = c->second;
        Bot* bot = &(bs.bot[botNumber]);
        bot->x = mainBlob->centroid.x;
        bot->y = mainBlob->centroid.y;
        bot->angle = 0.0;

        //now looking for a black blob inside the roi of this blob

        cvSetImageROI(seg_black, cvRect(mainBlob->minx, mainBlob->miny, mainBlob->maxx - mainBlob->minx, mainBlob->maxy - mainBlob->miny));
        IplImage* tmp = cvCreateImage(cvGetSize(seg_black), 8, 1);
        IplImage* labelImg = cvCreateImage(cvGetSize(seg_black), IPL_DEPTH_LABEL, 1);
        cvb::CvBlobs black_blobs;
        cvCopy(seg_black, tmp, NULL);
        cvZero(labelImg);
        cvLabel(tmp, labelImg, black_blobs);

        int area = 0;
        int n = 0;
        CvBlobs::const_iterator cb;
        for (CvBlobs::const_iterator it=black_blobs.begin(); it!=black_blobs.end(); ++it)
        {
            n++;
            if(it->second->area > area)
            {
                area = it->second->area;
                cb = it;
            }
        }

        //if black blob found
        if(n > 0)
        {
            CvBlob* blackBlob = cb->second;
//            bs.angle = cvb::cvAngle(cb->second);
//            cvCircle(cam.rgbimg, cvPoint(blackBlob->centroid.x + mainBlob->minx, blackBlob->centroid.y + mainBlob->miny), 2, cvScalar(255,255,255),3);
//            qDebug() << cb->second->m11<< " " << cb->second->m02<< " " << cb->second->m20;

            double xdiff = blackBlob->centroid.x + mainBlob->minx - mainBlob->centroid.x;
            double ydiff = blackBlob->centroid.y + mainBlob->miny - mainBlob->centroid.y;

            double angleFromDiff = atan2(ydiff, xdiff);
            angleFromDiff += CV_PI;
            while(angleFromDiff < -CV_PI)
                angleFromDiff += 2*CV_PI;
            while(angleFromDiff > CV_PI)
                angleFromDiff -= 2*CV_PI;
//            qDebug() << angleFromDiff;
            bot->angle = angleFromDiff;
            bot->isVisible = true;

//            qDebug() << "Angle is " << cvb::cvAngle(cb->second);
        }

        cvResetImageROI(seg_black);
        cvReleaseImage(&tmp);
        cvReleaseImage(&labelImg);
    }
}

void FeatureDetection::printBot(CamCapture &cam, int num)
{
    CvPoint p1 = cvPoint(bs.bot[num].x, bs.bot[num].y);
    CvPoint p2;
    p2.x = p1.x + 30.0*cos(bs.bot[num].angle);
    p2.y = p1.y + 30.0*sin(bs.bot[num].angle);

    cvLine(cam.rgbimg, p1, p2, cvScalar(255,255,255), 3);
}

void FeatureDetection::updateBeliefState(CamCapture &cam)
{
    getBlobs(cam);
    //need to see configuration. right now assuming
    //5 bots with colors red, orange, purple, green and pink

    getPosition(blobs_orange, 0);
    getPosition(blobs_purple, 1);
    getPosition(blobs_red, 2);
    getPosition(blobs_cyan, 3);
    getPosition(blobs_yellow, 4);

//    CvPoint p[3];
//    p[0] = cvPoint(100,100);
//    p[1] = cvPoint(200,200);
//    p[2] = cvPoint(300,300);

//    for(int i = 0; i <3; ++i)
//        cvCircle(cam.rgbimg, p[i], 5, cvScalar(200,0,0), 4);
    for(int i = 0; i < 5; ++i)
    {
        if(bs.bot[i].isVisible)
            printBot(cam, i);
    }

}
