#ifndef COLORDETECTION_H
#define	COLORDETECTION_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

namespace SniperBot
{
    /** ColorDetector Class
     * Purpose: To grab screen captures from a USB camera and detect a specific color. The
     * coordinates of the color are returned through the parameters passed in.
     */
    class ColorDetector
    {
    public:
        /** Error code for no error */
        static const int ERROR_NONE = 0;
        
        /** Error code for if the camera cannot be read */
        static const int ERROR_CANNOT_READ_CAMERA = 1;
        
        /** The default width code for the screen */
        static const long DEFAULT_WINDOW_WIDTH = 0;
        
        /** Red color code */
        static const int RED = 1;
        
        /** Blue color code */
        static const int BLUE = 2;
        
        /** Green color code */
        static const int GREEN = 3;
        
        /** Yellow color code */
        static const int YELLOW = 4;
        
    private:
        
        VideoCapture *cap; // holds a reference to a VideoCapture object used to grab screenshots
        int color;  // the code for the color to find
        bool showWindow;  // tells the findColorFromCam function to either show or hide
                          // what the camera sees
        long width;  // the width of the window
        bool drawCrosshair;  // tells the findColorFromCam function to draw a crosshair 
                             // at the x and y of the target
        bool showThreshold;  // tells the findColorFromCam function to either show or
                             // hide the threshold image
        
    public:
        
        /** The getScreenSize function grabs a screenshot from the camera an gets the width
         *  and height of the capture.
         * @param cap a reference to the video capture object used to connect to the camera
         * @return a Point object that stores the width and height of the screen capture
         */
        static Point getScreenSize(VideoCapture &cap)
        {
            Mat frame;  // Stores the screen capture
            bool bSuccess = cap.read(frame); // read a new frame from video

            if (!bSuccess) //if a screen capture was not successful
                return Point(0, 0);
            else  // if a screen capture was successful
                return Point(frame.cols, frame.rows);
	    }
        
        /** Constructor to create a ColorDetector object
         * @param cap a reference to a VideoCapture object used to grab screenshots
         * @param color the code for the color to find
         * @param showWindow tells the function to either show or hide what the camera sees
         * @param width the width of the window
         * @param drawCrosshair tells the function to draw a crosshair at the x and y of the color
         * @param showThreshold tells the function to either show or hide the threshold image
         */
        ColorDetector(VideoCapture &cap, int color, bool showWindow = false,
            long width = ColorDetector::DEFAULT_WINDOW_WIDTH, bool drawCrosshair = true,
            bool showThreshold = false);
        
        /** Gets the VideoCapture object
         * @return the VideoCapture object
         */
        VideoCapture *getVideoCapture();
        
        /** Gets the target color code
         * @return the target color code
         */
        int getColor();
        
        /** Sets the target color code
         * @param value the target color code
         */
        void setColor(int value);
        
        /** Gets if the window will be shown
         * @return if the window will be shown
         */
        bool getShowWindow();
        
        /** Sets if the window will show
         * @param value should the window be shown
         */
        void setShowWindow(bool value);
        
        /** Gets the width of the capture screen. 0 is default width.
         * @return the width of the capture screen
         */
        long getWidth();
        
        /** Sets the width of the capture screen
         * @param value the width of the capture screen
         */
        void setWidth(long value);
        
        /** Gets if the crosshair should be drawn
         * @return should the crosshair be draw
         */
        bool getDrawCrosshair();
        
        /** Sets if the crosshair should be drawn
         * @param value should the crosshair be drawn
         */
        void setDrawCrosshair(bool value);
        
        /** Gets if the threshold should be shown
         * @return should the threshold be drawn
         */
        bool getShowThreshold();
        
        /** Sets if the threshold window should be shown
         * @param value should the threshold window be shown
         */
        void setShowThreshold(bool value);
        
        /** The findColorFromCam function grabs a screen capture from the camera,
         * looks for the specified color, and sets the x and y parameters to the x 
         * and y coordinates of the color, if it is found.
         * @param x a reference to a variable to hold the x coordinate of the color
         * @param y a reference to a variable to hold the y coordinate of the color
         * @return an error code if an error occurs
         */
        int findColorFromCam(int &x, int &y);
    };
}

#endif	/* COLORDETECTION_H */

