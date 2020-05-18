#include <iostream>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "GPIO.h"
#include <math.h>
#include "ColorDetection.h"

using namespace cv;
using namespace std;
using namespace SniperBot;

// Robot control commands
const int STOP = 0;  // Stops the robot from moving
const int MOVE_FORWARD = 1;  // Moves the robot forward
const int MOVE_BACKWARDS = 2;  // Moves the robot backwards
const int TURN_LEFT = 3;  // Rotates the robot left
const int TURN_RIGHT = 4;  // Rotates the robot right
const int LOOK_LEFT = 5;  // Turns the camera left
const int LOOK_RIGHT = 6;  // Turns the camera right
const int LOOK_UP = 7;  // Turns the camera up
const int LOOK_DOWN = 8;  // Turns the camera down
const int START_FIRING = 9;  // Starts firing the laser
const int STOP_FIRING = 10;  // Stops firing the laser
const int CENTER_CAMERA = 11;  // Returns the camera to the center position

// Robot States
const int STATE_IDOL = 0;  // Robot not doing anything
const int STATE_SEARCHING = 1;  // Robot looking for the target color
const int STATE_AVOIDING_LEFT = 2;  // Robot is turning left to avoid an object
const int STATE_AVOIDING_RIGHT = 3;  // Robot is turning right to avoid an object
const int STATE_TARGETING = 4;  // Robot found an target and is aiming at it

// GPIO Variables
string data0Pin = "4";  // Bit 0 of the data bus
string data1Pin = "17";  // Bit 1 of the data bus
string data2Pin = "27";  // Bit 2 of the data bus
string data3Pin = "22";  // Bit 3 of the data bus
string trigPin = "12";  // Trigger pin for telling the Arduino there is a command being sent
string leftUSPin = "5";  // Data pin for the left ultrasonic sensor
string rightUSPin = "6";  // Data pin for the right ultrasonic sensor
string frontUSPin = "13";  // Data pin for the front ultrasonic sensor
GPIO *trig = new GPIO(trigPin);  // Connects to trigger pin
GPIO *data0 = new GPIO(data0Pin);  // Connects to data 0 pin
GPIO *data1 = new GPIO(data1Pin);  // Connects to data 1 pin
GPIO *data2 = new GPIO(data2Pin);  // Connects to data 2 pin
GPIO *data3 = new GPIO(data3Pin);  // Connects to data 3 pin
GPIO *leftUS = new GPIO(leftUSPin);  // Connects to left ultrasonic pin
GPIO *rightUS = new GPIO(rightUSPin);  // Connects to right ultrasonic pin
GPIO *frontUS = new GPIO(frontUSPin);  // Connects to front ultrasonic pin

ColorDetector *cd;  // Used to detect color from the camera
VideoCapture cap;  // Used to grab screenshots from the camera
Rect targetArea;  // Rectangle specifying where the color object should be for the robot to start firing
bool usLeftState;  // stores if the left ultrasonic sensor pin is high or not
bool usRightState;  // stores if the right ultrasonic sensor pin is high or not
bool usFrontState;  // stores if the front ultrasonic sensor pin is high or not
int state;  // holds the robot state

/** Sends a 4 bit command to the Arduino using 4 GPIO pins
 * @param data The data to be sent to the Arduino. This value is converted to binary
 * in order to put the data on the 4 data pins.
 */
void sendCommand(int data)
{
	string bits[4];  // holds the 4 data bits
	int d = data;  // copy of the value passed in
	
	// Check that data is in bounds on a 4 bit number
	if(data > 15)
            data = 15;
	else if(data < 0)
            data = 0;
	
        // Convert the data integer to binary and assign the 4 bits to the 4 element array
	for(int i = 3; i >= 0; --i)
	{
            if(d >= pow(2, i))
            {
                bits[i] = "1";
                d -= pow(2, i);
            }
            else
            {
                bits[i] = "0";
            }
	}
	
	trig->setval_gpio("0");  // clears the trigger pin
	data0->setval_gpio(bits[0]);  // sets data bit 0
	data1->setval_gpio(bits[1]);  // sets data bit 1
	data2->setval_gpio(bits[2]);  // sets data bit 2
	data3->setval_gpio(bits[3]);  // sets data bit 3
	trig->setval_gpio("1");  // Set the trigger pin high
}

/** Causes the program to pause for the specified milliseconds.
 * @param milli the number of milliseconds to pause
 */
void msleep(int milli)
{
    for(int i = 0; i < milli; ++i)
    {
        //usleep(1000);  // pause for 1000 microseconds (1 millisecond)
    }
}

/** Sets up the GPIO pins on the Raspberry Pi.*/
void setupGPIO()
{
    // Tell the Raspberry Pi which pins are being used
    trig->export_gpio();
    data0->export_gpio();
    data1->export_gpio();
    data2->export_gpio();
    data3->export_gpio();
    leftUS->export_gpio();
    rightUS->export_gpio();
    frontUS->export_gpio();
    
    // Set the pins to either input or output
    trig->setdir_gpio("out");
    data0->setdir_gpio("out");
    data1->setdir_gpio("out");
    data2->setdir_gpio("out");
    data3->setdir_gpio("out");
    leftUS->setdir_gpio("in");
    rightUS->setdir_gpio("in");
    frontUS->setdir_gpio("in");
}

/** Opens the camera, gets the size of the screen captures, and sets up the target area.
 * @return error code, if any
 */
int setupCamera()
{
    cap.open(0);  // open the first camera connected
    
    // If the camera is successful when opening, return an error code
    if(!cap.isOpened())
    	return 1;
    // If the camera opened successfully...
    else
    {
        // Gets the size of the screen from the camera
        Point size = SniperBot::ColorDetector::getScreenSize(cap);

        int targetWidth = 250;  // width of the target area
        int targetHieght = 250;  // height of the target area

        // Set the coordinates of the target area to position it in the middle of the camera's view
        targetArea.x = size.x / 2 - (targetWidth / 2);
        targetArea.y = size.y / 2 - (targetHieght / 2);
        targetArea.width = targetWidth;
        targetArea.height = targetHieght;
    }
	
    return 0;  // Return no error
}

/** Reads the states of the 3 ultrasonic sensor pins to determine if there are an object
 * within the collision threshold. */
void getUltrasonicStates()
{
    string left, right, front;  // holds the 3 ultrasonic sensor states

    // Get the states of the 3 ultrasonic pins
    leftUS->getval_gpio(left);
    rightUS->getval_gpio(right);
    frontUS->getval_gpio(front);

    // Sets the global ultrasonic state variables
    left == "1" ? usLeftState = true : usLeftState = false;
    right == "1" ? usRightState = true : usRightState = false;
    front == "1" ? usFrontState = true : usFrontState = false;
}

/** The program's starting point */
int main()
{
    int x, y;  // holds the x and y of the target. these are set to -1 if no target is found.
    bool xTargeted, yTargeted;  // flag for if the target is within the target area
    int targetColor = ColorDetector::GREEN;
    cd = new ColorDetector(cap, targetColor);
    setupGPIO();  // Setup the GPIO pins
    
    int camError = setupCamera();  // Setup the camera and target area
    
    // if the camera could not be setup, show an error message and end the program
    if(camError)
    {
    	cout << "Error: There was a problem setting up the camera." << endl;
    	return 1;
    }
    
    sendCommand(MOVE_FORWARD);  // Start the robot by telling it to move forward
    state = STATE_SEARCHING;  // set state to looking for target
    
    // main robot logic loop
    while(true)
    {
    	getUltrasonicStates(); // Determines if there are any objects in collision range
    	//cout << usLeftState << " " << usRightState << " " << usFrontState << endl;
    	
        // if the robot is searching...
    	if(state == STATE_SEARCHING)
    	{
            // If object detected in front
            if(usFrontState)
            {
                sendCommand(STOP); // Stop
                
                // Rotate left or right
                if(!usLeftState)
                {
                    sendCommand(TURN_LEFT);	// Turn left to avoid object in front
                    state = STATE_AVOIDING_LEFT; // Set state to avoiding object
                }
                else if(!usRightState)
                {
                    sendCommand(TURN_RIGHT);	// Turn right to avoid object in front
                    state = STATE_AVOIDING_RIGHT; // Set state to avoiding object
                }
            }
            // If object detected to the right
            else if(usRightState)
            {
                sendCommand(TURN_LEFT); // Turn left to avoid object on right
                state = STATE_AVOIDING_LEFT; // Set state to avoiding object
            }
            // If object detected to the left
            else if(usLeftState)
            {
                sendCommand(TURN_RIGHT); // Turn right to avoid object on left
                state = STATE_AVOIDING_RIGHT; // Set state to avoiding object
            }
	        
            // Look for target color
            cd->findColorFromCam(x, y);
            //cout << x << "  " << y << endl;

            // If target color is detected
            if(x != -1 && y != -1)
            {
                sendCommand(STOP);  // Stop
                state = STATE_TARGETING;  // Set state to targeting
            }
        }
        // If the robot is turning right to avoid an object
        else if(state == STATE_AVOIDING_RIGHT)
        {
            // If no object are detected to the left or front
            if(!usLeftState && !usFrontState)
            {
                // Stop turning right and start searching
                sendCommand(CENTER_CAMERA);
                sendCommand(MOVE_FORWARD);
                state = STATE_SEARCHING;
            }
        }
        // If the robot is turning left to avoid an object
        else if(state == STATE_AVOIDING_LEFT)
        {
            // If no object are detected to the right or front
            if(!usRightState && !usFrontState)
            {
                // Stop turning left and start searching
                sendCommand(CENTER_CAMERA);
                sendCommand(MOVE_FORWARD);
                state = STATE_SEARCHING;
            }
        }
        // If the robot is aiming at a target
        else if(state == STATE_TARGETING)
        {
            // Look for target color
            cd->findColorFromCam(x, y);
            //cout << "tx:" << targetArea.x << " tw:" << targetArea.width << " x:" << x << endl;

            // If no object is detected
            if(x == -1 && y == -1)
            {
                // tell the robot to start searching again
                state = STATE_SEARCHING;
                sendCommand(CENTER_CAMERA);
                sendCommand(STOP_FIRING);
                sendCommand(MOVE_FORWARD);
            }
            // if an object is detected
            else
            {
                // Clear the target flags. They are set if below if the x and y of
                // the target is within the target area.
                xTargeted = false;
                yTargeted = false;

                // If the target x is to the left of the target area
                if(x < targetArea.x)
                    sendCommand(LOOK_LEFT);
                // If the target x is to the right of the target area
                else if(x > targetArea.x + targetArea.width)
                    sendCommand(LOOK_RIGHT);
                // target x is in the target area
                else
                    xTargeted = true;

                // If the target y is above the target area
                if(y < targetArea.y)
                    sendCommand(LOOK_UP);
                // If the target y is below the target area
                else if(y > targetArea.y + targetArea.height)
                    sendCommand(LOOK_DOWN);
                // target y is in the target area
                else
                    yTargeted = true;

                // If both target x and y are in the target area
                if(xTargeted && yTargeted)
                    sendCommand(START_FIRING);  // fire at target
                // target x and y are not in the target area
                else
                    sendCommand(STOP_FIRING);  // stop firing
            }
        }
    	
    	if(waitKey(30) == 27) break;  // wait for the user to press the escape key
        
        return 0;  // return no error and end the program
    }
}