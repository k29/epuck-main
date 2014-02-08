#ifndef CAMCAPTURE_H
#define CAMCAPTURE_H

#include <opencv2/opencv.hpp>
#include "imgproc.h"

// #define CAMCAPTURE_DEBUG
/*
HOW TO USE THIS CLASS
1. Make object of class.
2. Call init().
3. When you need image, call getImage().
   It will store the images in
   rgbimg, bayerimg
4. ** IF YOU DON'T WANT LOOKUP TABLES, SEND false AS ARGUMENT TO CONSTRUCTOR. THEN YOU CANNOT USE isRed() etc. **
There are 3 images:
rgbimg - default one, to be used normally. it may be reduced or not.
rgbimg_full - full image, direct from camera. does not have any functions to check colors (no lut lookup functions for this)
rgbimg_small - 2nd reduced image. to be used for blob detection. associated lut functions have _small in front of them
*/


enum CamError {CAM_SUCCESS = 1, CAM_FAILURE = 0};
class CamCapture
{
private:
    bool isInit;
    bool doLUT;
    unsigned char* lut_red;
    unsigned char* lut_yellow;
    unsigned char* lut_cyan;
    unsigned char* lut_c1;
    unsigned char* lut_c2;
    unsigned char* lut_black;
    unsigned char* lut_green;
    unsigned char* lut_maroon;
    char* fileName;
    int width_var;
    int height_var;
    CvCapture* capture;
    bool loadLUT(Color color);
    void showSegmentation();
    //TODO:
    //makeinfoimg is our own implementation of a lut
    //OpenCV has a function called LUT that does this job
    //Maybe look into it?
 //   void makeinfoimg(IplImage*, unsigned char*);

public:
    IplImage* rgbimg;
    IplImage* showSeg;
    inline bool isRed(int i, int j)
    {
        if(lut_red[returnPixel3C(rgbimg, i, j, 2)
            |(returnPixel3C(rgbimg, i, j, 1)<<8)
            |(returnPixel3C(rgbimg, i, j, 0)<<16)])
            return true;
        else
            return false;

    }
    inline bool isCyan(int i, int j)
    {
        if(lut_cyan[returnPixel3C(rgbimg, i, j, 2)
            |(returnPixel3C(rgbimg, i, j, 1)<<8)
            |(returnPixel3C(rgbimg, i, j, 0)<<16)])
            return true;
        else
            return false;

    }
    inline bool isYellow(int i, int j)
    {
        if(lut_yellow[returnPixel3C(rgbimg, i, j, 2)
            |(returnPixel3C(rgbimg, i, j, 1)<<8)
            |(returnPixel3C(rgbimg, i, j, 0)<<16)])
            return true;
        else
            return false;

    }
    inline bool isC1(int i, int j)
    {
        if(lut_c1[returnPixel3C(rgbimg, i, j, 2)
            |(returnPixel3C(rgbimg, i, j, 1)<<8)
            |(returnPixel3C(rgbimg, i, j, 0)<<16)])
            return true;
        else
            return false;

    }
    inline bool isC2(int i, int j)
    {
        if(lut_c2[returnPixel3C(rgbimg, i, j, 2)
            |(returnPixel3C(rgbimg, i, j, 1)<<8)
            |(returnPixel3C(rgbimg, i, j, 0)<<16)])
            return true;
        else
            return false;
    }
    inline bool isBlack(int i, int j)
    {
        if(lut_black[returnPixel3C(rgbimg, i, j, 2)
            |(returnPixel3C(rgbimg, i, j, 1)<<8)
            |(returnPixel3C(rgbimg, i, j, 0)<<16)])
            return true;
        else
            return false;
    }
    inline bool isGreen(int i, int j)
    {
        if(lut_green[returnPixel3C(rgbimg, i, j, 2)
            |(returnPixel3C(rgbimg, i, j, 1)<<8)
            |(returnPixel3C(rgbimg, i, j, 0)<<16)])
            return true;
        else
            return false;
    }
    inline bool isMaroon(int i, int j)
    {
        if(lut_maroon[returnPixel3C(rgbimg, i, j, 2)
            |(returnPixel3C(rgbimg, i, j, 1)<<8)
            |(returnPixel3C(rgbimg, i, j, 0)<<16)])
            return true;
        else
            return false;
    }

    CamCapture(bool param = true, char *file = NULL);
    ~CamCapture();
    CamError init(int nCamera = -1);
    CamError getImage();
    inline int width(){     return width_var;}
    inline int height(){    return height_var;}
};
#endif
