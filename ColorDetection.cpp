#include "ColorDetection.h"
#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

namespace SniperBot
{
    // Constructor
    ColorDetector::ColorDetector(VideoCapture &c, int color, bool showWindow,
        long width, bool drawCrosshair, bool showThreshold)
    {
        this->cap = &c;
        this->color = color;
        this->showWindow = showWindow;
        this->width = width;
        this->drawCrosshair = drawCrosshair;
        this->showThreshold = showThreshold;
    }
    
    VideoCapture *ColorDetector::getVideoCapture() { return cap; }
        
    // getColor function
    int ColorDetector::getColor() { return color; }

    // setColor function
    void ColorDetector::setColor(int value) { color = value; }

    // getShowWindow function
    bool ColorDetector::getShowWindow() { return showWindow; }

    // setShowWindow function
    void ColorDetector::setShowWindow(bool value) { showWindow = value; }

    // getWidth function
    long ColorDetector::getWidth() { return width; }

    // setWidth function
    void ColorDetector::setWidth(long value) { width = value; }

    // getDrawCrosshair function
    bool ColorDetector::getDrawCrosshair() { return drawCrosshair; }

    // setDrawCrosshair function
    void ColorDetector::setDrawCrosshair(bool value) { drawCrosshair = value; }

    // getShowThreshold function
    bool ColorDetector::getShowThreshold() { return showThreshold; }

    // setShowThreshold function
    void ColorDetector::setShowThreshold(bool value) { showThreshold = value; }

    int ColorDetector::findColorFromCam(int &x, int &y)
    {
        Mat imgOriginal;  // holds the image matrix of the camera capture

        bool bSuccess = (*cap).read(imgOriginal); // read a new frame from camera

        //if could not read from camera, return error
        if (!bSuccess)
            return ERROR_CANNOT_READ_CAMERA;
        
        // if width is not the default width, resize the window keeping the aspect ratio
        if(width > 0)
            resize(imgOriginal, imgOriginal,
                    Size(width, (int)(imgOriginal.rows * (width / (float)imgOriginal.cols))));
        
        Mat imgHSV;  // stores the HSV color version of the original camera capture
        
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        
        Mat imgThresholded;
        // This needs to be tested in a control environment to acquire predefined colors and their values. Colors can be selected selected from switch case.
        switch(color)
        {
            case 1:  // Red
                inRange(imgHSV, Scalar(170, 150, 60), Scalar(179, 255, 255), imgThresholded); //Threshold the image
                break;
            case 2:  // Blue
                inRange(imgHSV, Scalar(72, 63, 28), Scalar(179, 255, 255), imgThresholded); //Threshold the image
                break;
            case 3:  // Green
                inRange(imgHSV, Scalar(0, 107, 102), Scalar(179, 255, 187), imgThresholded); //Threshold the image
                break;
            case 4:  // Yellow
                inRange(imgHSV, Scalar(19, 0, 169), Scalar(44, 255, 255), imgThresholded); //Threshold the image
                break;
        }
        
        //morphological opening (removes small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        //morphological closing (removes small holes from the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        
        //Calculate the moments of the thresholded image
        Moments oMoments = moments(imgThresholded);
        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;
        
        // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
        if (dArea > 10000)
        {
            //calculate the position of the target object
            int posX = dM10 / dArea;
            int posY = dM01 / dArea;
            x = posX;
            y = posY;
            
            if(drawCrosshair)  // Draw a crosshair
            {
                circle(imgOriginal, Point(posX, posY), 15, Scalar(0, 255, 0));
                line(imgOriginal, Point(posX, posY - 20), Point(posX, posY - 5), Scalar(0, 255, 0));
                line(imgOriginal, Point(posX, posY + 20), Point(posX, posY + 5), Scalar(0, 255, 0));
                line(imgOriginal, Point(posX - 20, posY), Point(posX - 5, posY), Scalar(0, 255, 0));
                line(imgOriginal, Point(posX + 20, posY), Point(posX + 5, posY), Scalar(0, 255, 0));
            }
        }
        else  // No target object detected
        {
            x = -1;
            y = -1;
        }
        
        // Show window
        if(showWindow) imshow("Original", imgOriginal); //show the original image
        
        // Show threshold window
        if(showThreshold) imshow("Threshold", imgThresholded);
        
        return ERROR_NONE;
    }
}