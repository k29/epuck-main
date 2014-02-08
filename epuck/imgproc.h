#ifndef IMGPROC_H
#define IMGPROC_H
//WARNING: imgproc contains dangerous #defines, must be places after all other headers, otherwise causes errors
#include <opencv2/opencv.hpp>
//THIS FILE CONTAINS MACROS AND INLINE FUNCTIONS USED IN MOST IMAGE PROCESSING CLASSES
typedef unsigned char uchar;

enum Color { C_RED, C_YELLOW, C_CYAN, C_C1, C_C2, C_BLACK, C_GREEN, C_MAROON };

#define returnPixel1C(image, x, y) ((uchar*)(image->imageData + image->widthStep*(y)))[x]
#define returnPixel3C(image, x, y, color) ((uchar*)(image->imageData + image->widthStep*(y)))[(x)*3 + color]

//BGR
inline void setPixel3C(CvScalar &pixel, uchar blue, uchar green, uchar red)
{
    pixel.val[0] = blue;
    pixel.val[1] = green;
    pixel.val[2] = red;
    return;
}

inline uchar pixelColor3C(IplImage* image, int x, int y, int color)
{
    return ((uchar*)(image->imageData + image->widthStep*y))[x*3 + color];
}

inline void pixelColor3C(IplImage* image, int x, int y, int *rgb)
{
    rgb[0] = ((uchar*)(image->imageData + image->widthStep*y))[x*3 + 2];
    rgb[1] = ((uchar*)(image->imageData + image->widthStep*y))[x*3 + 1];
    rgb[2] = ((uchar*)(image->imageData + image->widthStep*y))[x*3 + 0];
    return;
}

inline uchar pixelColor1C(IplImage* image, int x, int y)
{
    return ((uchar*)(image->imageData + image->widthStep*y))[x];
}

#endif
