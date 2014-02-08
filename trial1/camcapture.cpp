#include <opencv2/opencv.hpp>
#include "camcapture.h"
#include "imgproc.h"
#include <cstdio>
#include <cstring>

//pass true as parameter to load luts, false to skip loading luts
CamCapture::CamCapture(bool param, char* file)
{
	isInit = false;
    doLUT = param;
    capture = NULL;
    if(file!=NULL)
    {
        fileName = new char[30];
        strcpy(fileName, file);
    }
    else
    {
        fileName = NULL;
    }
}



CamCapture::~CamCapture()
{
	if(isInit==false)
		return;
    // Stop capturing images
}



CamError CamCapture::init(int nCamera)
{
    if(isInit==true)
        return CAM_SUCCESS;

    //Loading lookup table
    if(doLUT == true)
    {
        printf("Loading lookup tables...");
        if(loadLUT(C_RED)==false||loadLUT(C_YELLOW)==false||loadLUT(C_CYAN)==false||loadLUT(C_C1)==false||loadLUT(C_C2)==false||loadLUT(C_BLACK)==false||loadLUT(C_GREEN)==false||loadLUT(C_MAROON)==false)
        {
            printf("Unable to open LUT\n");
            return CAM_FAILURE;
        }
        printf("Loaded.\n");
    }
    //Initializing camera
    if(fileName == NULL)
    {
        capture = cvCaptureFromCAM(nCamera);
        if(!capture)
            return CAM_FAILURE;
//        cvSetCaptureProperty(capture, CV_CAP_PROP_OPENNI_FOCAL_LENGTH, 40);
        IplImage* tempImage = cvQueryFrame(capture);
        width_var = tempImage->width;
        height_var = tempImage->height;
    }
    else
    {
        IplImage* tempImage = cvLoadImage(fileName);
         if(!tempImage)
            return CAM_FAILURE;
        rgbimg = cvCreateImage(cvSize(640, 480), 8, 3);
        cvResize(tempImage, rgbimg);
       
        width_var = rgbimg->width;
        height_var = rgbimg->height;
    }
    
    // Size variables cannot dynamically changed. Must be defined in constructor
    // width_var_full = rawImage.GetCols();
    // height_var_full = rawImage.GetRows();

#ifdef CAMCAPTURE_DEBUG
    showSeg = cvCreateImage(cvSize(width_var, height_var), 8, 3);
#endif

    // width_var = rgbimg->width;
    // height_var = rgbimg->height;
    isInit = true;
    return CAM_SUCCESS;

}



CamError CamCapture::getImage()
{
	if(isInit==false)
		return CAM_FAILURE;
    // Start capturing images
    if(fileName == NULL)
    {
        rgbimg = cvQueryFrame(capture);
        if(!rgbimg)
        	return CAM_FAILURE;
    }
    else
    {
        cvReleaseImage(&rgbimg);
        IplImage* tempImage = cvLoadImage(fileName);
        if(!tempImage)
            return CAM_FAILURE;
        rgbimg = cvCreateImage(cvSize(640, 480), 8, 3);
        cvResize(tempImage, rgbimg);
        cvReleaseImage(&tempImage);
    }

//    cvMirror(rgbimg, rgbimg, 1);

#ifdef CAMCAPTURE_DEBUG
    if(doLUT==true)
        showSegmentation();
#endif

    return CAM_SUCCESS;
}



bool CamCapture::loadLUT(Color color)
{
    //Loads lut in variable lut
    FILE *fp;
    uchar** lut_address;
    char file[20];
    switch(color)
    {
        case C_RED: 
        lut_address = &lut_red;
        strcpy(file, "red.lut");
        break;

        case C_YELLOW:
        lut_address = &lut_yellow;
        strcpy(file, "yellow.lut");
        break;

        case C_CYAN:
        lut_address = &lut_cyan;
        strcpy(file, "cyan.lut");
        break;

        case C_C1:
        lut_address = &lut_c1;
        strcpy(file, "c1.lut");
        break;

        case C_C2:
        lut_address = &lut_c2;
        strcpy(file, "c2.lut");
        break;

        case C_BLACK:
        lut_address = &lut_black;
        strcpy(file, "black.lut");
        break;        

        case C_GREEN:
        lut_address = &lut_green;
        strcpy(file, "green.lut");
        break;

        case C_MAROON:
        lut_address = &lut_maroon;
        strcpy(file, "maroon.lut");
        break;

    }
    fp = fopen(file,"rb");
    if(!fp)
        return false;

    *lut_address = new uchar [256*256*256];
    if(!fread(*lut_address,sizeof(uchar),256*256*256,fp))
        return false;

    fclose(fp);

    return true;
}



void CamCapture::showSegmentation()
{
    CvScalar pixel; 
    for (int x = 0; x < width_var; ++x)
    {
        for (int y = 0; y < height_var; ++y)
        {
            
            if(isRed(x,y))
            {
                setPixel3C(pixel,0,0,255);
            }
            else if(isCyan(x, y))
            {
                setPixel3C(pixel,255,0,0);
            }
            else if(isYellow(x, y))
            {
                setPixel3C(pixel,0,255,255);
            }
            else if(isC1(x, y))
            {
                setPixel3C(pixel,0,255,0);
            }
            else if(isC2(x, y))
            {
                setPixel3C(pixel, 255, 255, 255);
            }
            else if(isBlack(x, y))
            {
                setPixel3C(pixel, 127, 127, 127);   
            }
            else if(isGreen(x, y))
            {
                setPixel3C(pixel, 0, 127, 0);   
            }
            else if(isMaroon(x, y))
            {
                setPixel3C(pixel, 0, 0, 127);   
            }
            else
            {
                setPixel3C(pixel,0,0,0);
            }

            cvSet2D(showSeg, y, x, pixel);
        }
    }
}
