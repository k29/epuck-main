#include "featuredetection.h"
#include <fstream>
#include <QDebug>
#include <vector>

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
    seg_maroon = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), 8, 1);
    labelImg = cvCreateImage(cvSize(IMAGE_WIDTH, IMAGE_HEIGHT), IPL_DEPTH_LABEL, 1);
}



//For parallel segmenting
class SegmentImages_all {
    IplImage* my_seg_yellow;
    IplImage* my_seg_cyan;
    IplImage* my_seg_red;
    IplImage* my_seg_maroon;
    CamCapture* my_cam;
public:
    void operator()( const blocked_range2d<size_t>& r ) const {
        for( size_t x=r.rows().begin(); x!=r.rows().end(); ++x )
        {
            for( size_t y=r.cols().begin(); y!=r.cols().end(); ++y )
            {
                if(my_cam->isYellow(x, y))
                    returnPixel1C(my_seg_yellow, x, y) = 255;

                if(my_cam->isCyan(x, y))
                    returnPixel1C(my_seg_cyan, x, y) = 255;

                if(my_cam->isRed(x, y))
                    returnPixel1C(my_seg_red, x, y) = 255;

                if(my_cam->isMaroon(x, y))
                    returnPixel1C(my_seg_maroon, x, y) = 255;
            }
        }
    }
    SegmentImages_all(IplImage* &seg_yellow, IplImage* &seg_cyan, IplImage* &seg_red, IplImage* &seg_maroon, CamCapture &cam) :
        my_seg_yellow(seg_yellow), my_seg_cyan(seg_cyan), my_seg_red(seg_red), my_seg_maroon(seg_maroon), my_cam(&cam)
    {}
};



void FeatureDetection::getBlobs(CamCapture &cam)
{
    cvZero(seg_yellow);
    cvZero(seg_cyan);
    cvZero(seg_red);
    cvZero(seg_maroon);
    parallel_for( blocked_range2d<size_t>(line_x1, line_x2, 16, line_y1, line_y2, 32),
                  SegmentImages_all(seg_yellow,seg_cyan,seg_red,seg_maroon, cam) );
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
    cvLabel(seg_maroon, labelImg, blobs_maroon);
    cvFilterByArea(blobs_maroon, BLOB_MIN_AREA, 1000000);
#ifdef RENDER_BLOBS
    cvRenderBlobs(labelImg, blobs_maroon, cam.rgbimg, cam.rgbimg);
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
//    CvBlobs::const_iterator c;
//    int nBlobs = 0;
//    int area = 0;
//    //get largest blob
//    for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it)
//    {
//        nBlobs++;
//        if(it->second->area > area)
//        {
//            area = it->second->area;
//            c = it;
//        }
//    }

//    bs.bot[botNumber].isVisible = false;

//    if(nBlobs)
//    {
//        CvBlob* mainBlob = c->second;
//        Bot* bot = &(bs.bot[botNumber]);
//        bot->x = mainBlob->centroid.x;
//        bot->y = mainBlob->centroid.y;
//        bot->angle = 0.0;

//        //now looking for a black blob inside the roi of this blob

//        cvSetImageROI(seg_black, cvRect(mainBlob->minx, mainBlob->miny, mainBlob->maxx - mainBlob->minx, mainBlob->maxy - mainBlob->miny));
//        IplImage* tmp = cvCreateImage(cvGetSize(seg_black), 8, 1);
//        IplImage* labelImg = cvCreateImage(cvGetSize(seg_black), IPL_DEPTH_LABEL, 1);
//        cvb::CvBlobs black_blobs;
//        cvCopy(seg_black, tmp, NULL);
//        cvZero(labelImg);
//        cvLabel(tmp, labelImg, black_blobs);

//        int area = 0;
//        int n = 0;
//        CvBlobs::const_iterator cb;
//        for (CvBlobs::const_iterator it=black_blobs.begin(); it!=black_blobs.end(); ++it)
//        {
//            n++;
//            if(it->second->area > area)
//            {
//                area = it->second->area;
//                cb = it;
//            }
//        }

//        //if black blob found
//        if(n > 0)
//        {
//            CvBlob* blackBlob = cb->second;
////            bs.angle = cvb::cvAngle(cb->second);
////            cvCircle(cam.rgbimg, cvPoint(blackBlob->centroid.x + mainBlob->minx, blackBlob->centroid.y + mainBlob->miny), 2, cvScalar(255,255,255),3);
////            qDebug() << cb->second->m11<< " " << cb->second->m02<< " " << cb->second->m20;

//            double xdiff = blackBlob->centroid.x + mainBlob->minx - mainBlob->centroid.x;
//            double ydiff = blackBlob->centroid.y + mainBlob->miny - mainBlob->centroid.y;

//            double angleFromDiff = atan2(ydiff, xdiff);
//            angleFromDiff += CV_PI;
//            while(angleFromDiff < -CV_PI)
//                angleFromDiff += 2*CV_PI;
//            while(angleFromDiff > CV_PI)
//                angleFromDiff -= 2*CV_PI;
////            qDebug() << angleFromDiff;
//            bot->angle = angleFromDiff;
//            bot->isVisible = true;

////            qDebug() << "Angle is " << cvb::cvAngle(cb->second);
//        }

//        cvResetImageROI(seg_black);
//        cvReleaseImage(&tmp);
//        cvReleaseImage(&labelImg);
//    }
}

void FeatureDetection::printBot(CamCapture &cam, int num)
{
    CvPoint p1 = cvPoint(bs.bot[num].x, bs.bot[num].y);
    CvPoint p2;
    p2.x = p1.x + 30.0*cos(bs.bot[num].angle);
    p2.y = p1.y + 30.0*sin(bs.bot[num].angle);
    char buf[20];
    sprintf(buf, "%d", num);
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1);
    cvLine(cam.rgbimg, p1, p2, cvScalar(50,50,50), 3);
    cvPutText(cam.rgbimg, buf, cvPoint(p1.x, p1.y - 20), &font, cvScalar(0,0,0));
    cvPutText(cam.rgbimg, buf, cvPoint(p1.x-1, p1.y - 21), &font, cvScalar(0,255,255));
    cvCircle(cam.rgbimg, p1, 3, cvScalar(255, 255, 255), 2);
}

FeatureDetection::customBlob FeatureDetection::getClosest(CvPoint2D64f p, std::vector<customBlob> v)
{
    double distance = 999999999.0;
    customBlob closestBlob;
    for(int i = 0; i < v.size(); ++i)
    {
        double tempDistance = getDistance(cvPoint(p.x, p.y), cvPoint(v[i].x, v[i].y));
        if(tempDistance < distance)
        {
            distance = tempDistance;
            closestBlob = v[i];
        }
    }
    return closestBlob;
}

int FeatureDetection::getRobotIDByColour(BlobColour largeColour, BlobColour smallColour)
{
    if(largeColour == BLOB_RED)
    {
        if(smallColour == BLOB_RED)
            return 0;
        else if(smallColour == BLOB_MAROON)
            return 1;
        else
            return 2;
    }
    else if(largeColour == BLOB_MAROON)
    {
        if(smallColour == BLOB_RED)
            return 3;
        else if(smallColour == BLOB_MAROON)
            return 4;
        else
            return 5;
    }
    else
    {
        if(smallColour == BLOB_RED)
            return 6;
        else
            return 7;
    }
}

void FeatureDetection::updateBeliefState(CamCapture &cam)
{
    getBlobs(cam);

    //need to see configuration. right now assuming
    //5 bots with colors red, orange, purple, green and pink
//    qDebug() << blobs_cyan.size();
    std::vector<customBlob> largeBlobs;
    std::vector<customBlob> smallBlobs;

    for(int i = 0; i < 3; ++i)
    {
        BlobColour bc;
        cvb::CvBlobs* blobs;
        if(i == 0)
        {
            blobs = &blobs_red;
            bc = BLOB_RED;
        }
        else if(i == 1)
        {
            blobs = &blobs_maroon;
            bc = BLOB_MAROON;
        }
        else
        {
            blobs = &blobs_yellow;
            bc = BLOB_YELLOW;
        }

        for (CvBlobs::const_iterator it=blobs->begin(); it!=blobs->end(); ++it)
        {
            customBlob blob;
            blob.colour = bc;
            blob.x = it->second->centroid.x;
            blob.y = it->second->centroid.y;
            if(it->second->area > 250)  //if more than 350, it is a large blob, otherwise small blob
            {
                largeBlobs.push_back(blob);
            }
            else
            {
                smallBlobs.push_back(blob);
            }
        }

    }


    for(int i = 0; i < NUMBOTS; ++i)
    {
        bs.bot[i].isVisible = false;
    }

    for (CvBlobs::const_iterator it=blobs_cyan.begin(); it!=blobs_cyan.end(); ++it)
    {
        customBlob l = getClosest(it->second->centroid, largeBlobs);
        customBlob s = getClosest(it->second->centroid, smallBlobs);

        int id = getRobotIDByColour(l.colour, s.colour);
        if(id >= NUMBOTS)
            continue;
        bs.bot[id].isVisible = true;
        bs.bot[id].x = it->second->centroid.x;
        bs.bot[id].y = it->second->centroid.y;
        double xdiff = l.x - s.x;
        double ydiff = l.y - s.y;

        double angleFromDiff = atan2(ydiff, xdiff);
        angleFromDiff += CV_PI;
        while(angleFromDiff < -CV_PI)
            angleFromDiff += 2*CV_PI;
        while(angleFromDiff > CV_PI)
            angleFromDiff -= 2*CV_PI;
        bs.bot[id].angle = angleFromDiff;

    }

    for(int i = 0; i < NUMBOTS; ++i)
    {
        if(bs.bot[i].isVisible)
            printBot(cam, i);
    }

}

void FeatureDetection::initBSSimulation()
{
    for(int i = 0; i < NUMBOTS; ++i)
    {
        bs.bot[i].isVisible = true;
        bs.bot[i].x = i*10 + 50;
        bs.bot[i].y = i*10 + 50;
        bs.bot[i].angle = CV_PI;
    }
}


//TODO: remove this copying of code
void FeatureDetection::printBotSimulation(CamCapture* cam, BeliefState bs)
{
    for(int num = 0; num < NUMBOTS; ++num)
    {
        cvCircle(cam->rgbimg, cvPoint(bs.bot[num].x, bs.bot[num].y), 5, cvScalar(255, 0, 0));
        CvPoint p1 = cvPoint(bs.bot[num].x, bs.bot[num].y);
        CvPoint p2;
        p2.x = p1.x + 30.0*cos(bs.bot[num].angle);
        p2.y = p1.y + 30.0*sin(bs.bot[num].angle);
        char buf[20];
        sprintf(buf, "%d", num);
        CvFont font;
        cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1);
        cvLine(cam->rgbimg, p1, p2, cvScalar(50,50,50), 3);
        cvPutText(cam->rgbimg, buf, cvPoint(p1.x, p1.y - 20), &font, cvScalar(0,0,0));
        cvPutText(cam->rgbimg, buf, cvPoint(p1.x-1, p1.y - 21), &font, cvScalar(0,255,255));
        cvCircle(cam->rgbimg, p1, 3, cvScalar(255, 255, 255), 2);
    }
}
