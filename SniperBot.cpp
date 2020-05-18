#include <Servo.h>

// Robot Commands
byte const STOP = 0;
byte const MOVE_FORWARD = 1;
byte const MOVE_BACKWARDS = 2;
byte const TURN_LEFT = 3;
byte const TURN_RIGHT = 4;
byte const LOOK_LEFT = 5;
byte const LOOK_RIGHT = 6;
byte const LOOK_UP = 7;
byte const LOOK_DOWN = 8;
byte const START_FIRING = 9;
byte const STOP_FIRING = 10;
byte const CENTER_CAMERA = 11;

float const DEG = PI / 180;  /** Constant used to convert radians to degrees */
int const RIGHT_WHEEL_FORWARD = 0;  /** value of the right wheel servo to go forward full speed */
int const RIGHT_WHEEL_BACKWARDS= 180;  /** value of the right wheel servo to go barckwards full speed */
int const LEFT_WHEEL_FORWARD = 180;  /** value of the left wheel servo to go forward full speed */
int const LEFT_WHEEL_BACKWARDS= 0;  /** value of the left wheel servo to go backwards full speed */
float const WHEEL_STOP = 90;  /** stopping value of both left and right wheel servos */
float speedScale;  /** used to change the speed of the robot wheels. value is from 0.0 to 1.0 */
int const MIN_PITCH = 45;  /** minimum pitch angle the camera can move */
int const MAX_PITCH = 135;  /** maximum pitch angle the camera can move */
int const MIN_YAW = 45;  /** minimum yaw angle the camera can move */
int const MAX_YAW = 135;  /** maximum yaw angle the camera can move */
int const PITCH_CENTER_ANGLE = 90;  /** center position of the pitch servo */
int const YAW_CENTER_ANGLE = 90;  /** center position of the yaw servo */

// Communication variables
byte pinDataBit0 = 40;  /** Pin for data bit 0 */
byte pinDataBit1 = 41;  /** Pin for data bit 1 */
byte pinDataBit2 = 42;  /** Pin for data bit 2 */
byte pinDataBit3 = 43;  /** Pin for data bit 3 */
byte pinTrig = 3;  /** pin for the data trigger */
byte pinLeftUSData = 30;  /** pin for the left ultrasonic sensor data */
byte pinRightUSData = 31;  /** pin for the right ultrasonic sensor data */
byte pinFrontUSData = 32;  /** pin for the front ultrasonic sensor data */
byte pinLaser = 9;  /** pin for controlling the laser */
bool isFiring = false;  /** flag for if the robot is currently firing the laser */

// Servo variables
byte pinLeftWheel = 7;  /** pin for the left wheel servo */
byte pinRightWheel = 52;  /** pin for the right wheel servo */
byte pinCameraPitch = 2;  /** pin for the camera pitch servo */
byte pinCameraYaw = 4;  /** pin for the camera yaw servo */ 
Servo leftWheel;  /** used to control the left wheel */
Servo rightWheel;  /** used to control the right wheel */
Servo cameraPitch;  /** used to control the camera pitch */
Servo cameraYaw;  /** user to control the camera yaw */
byte currentPitchAngle;  /** stores the current pitch angle of the camera */
byte currentYawAngle;  /** stores the current yaw angle of the camera */
byte cameraSpeed;  /** stores how many degrees the camera servos step each time they are moved */

// Ultrasonic sensor variables
byte pinLeftUS = 22;  /** data pin for the left ultrasonic sensor */
byte pinRightUS = 51;  /** data pin for the right ultrasonic sensor */
byte pinFrontUS = 53;  /** data pin for the front ultrasonic sensor */

/** The first function to run when the program starts. Used for initial setup. */
void setup()
{
  Serial.begin(9600);  // Start serial communication. this is used for debugging.
  
  noInterrupts();  // disable interrupts
  
  speedScale = 1;  // Set the robot wheel servos to move at full speed
  
  // Setup communication pins
  pinMode(pinDataBit0, INPUT);
  pinMode(pinDataBit1, INPUT);
  pinMode(pinDataBit2, INPUT);
  pinMode(pinDataBit3, INPUT);
  pinMode(pinTrig, INPUT);
  attachInterrupt(1, handleCommand, RISING);
  pinMode(pinLeftUSData, OUTPUT);
  pinMode(pinRightUSData, OUTPUT);
  pinMode(pinFrontUSData, OUTPUT);
  pinMode(pinLaser, OUTPUT);
  
  // Setup servos
  leftWheel.attach(pinLeftWheel);
  leftWheel.write(90);  // stop any rotation of the left wheel
  rightWheel.attach(pinRightWheel);
  rightWheel.write(90);  // stop any rotation of the right wheel
  cameraPitch.attach(pinCameraPitch);
  cameraYaw.attach(pinCameraYaw);
  currentPitchAngle = PITCH_CENTER_ANGLE;  // set the current camera pitch angle to center
  currentYawAngle = YAW_CENTER_ANGLE;  // set the current camera yaw angle to center
  cameraPitch.write(currentPitchAngle);  // center the camera pitch servo
  cameraYaw.write(currentYawAngle);  // center the camera yaw servo
  cameraSpeed = 2;  // set the camera to move in increments of 2 degrees
  
  interrupts();  // enable all interrupts
}

/** Runs after the setup function. This function executes an infinite number of times. */
void loop()
{
  moveForward(0);
  return;
  
  // Check for collisions
  checkCollisions(16);
  
  // If the robot is currently firing the laser...
  if(isFiring)
  {
    // flash the laser on and off 1 time
    digitalWrite(pinLaser, HIGH);
    tone(10, 100);
    delay(20);
    digitalWrite(pinLaser, LOW);
    noTone(10);
    delay(20);
  }
}

/** Checks the 3 ultrasonic sensors for objects within the provided distance. This function
    communicates the object detection data through 3 IO pins, 1 for each sensor. If an object
    is detected, the pin is set high. If an object is not detected, the pin is set low. To avoid
    accidental object detection, each sensor checks for an object 3 times. If an object is detected
    all 3 times, then the data pin is set high, otherwise, it is set low.
    @param cm the maximum distance (in centimeters) that an object can be to set the data pin high
*/
void checkCollisions(int cm)
{
  int numChecks = 3;  // number of times each sensor is checked for an object
  int count = 0;  // the count of how many times an object was detected
  
  // Check left sensor
  for(int i = 0; i < numChecks; ++i)
    // If an object is detected...
    if(getUltrasonicDistance(pinLeftUS) <= cm)
      ++count;
  // if all checks had an object detected...
  if(count == numChecks)
    digitalWrite(pinLeftUSData, HIGH);
  else
    digitalWrite(pinLeftUSData, LOW);
  
  // Check right sensor
  count = 0;
  for(int i = 0; i < numChecks; ++i)
    // If an object is detected...
    if(getUltrasonicDistance(pinRightUS) <= cm)
      ++count;
  // if all checks had an object detected...
  if(count == numChecks)
    digitalWrite(pinRightUSData, HIGH);
  else
    digitalWrite(pinRightUSData, LOW);
  
  // Check front sensor
  count = 0;
  for(int i = 0; i < numChecks; ++i)
    // If an object is detected...
    if(getUltrasonicDistance(pinFrontUS) <= cm)
      ++count;
  // if all checks had an object detected...
  if(count == numChecks)
    digitalWrite(pinFrontUSData, HIGH);
  else
    digitalWrite(pinFrontUSData, LOW);
}

/** Gets the distance of an object using an ultrasonic sensor. This function calculates the distance
    in centimeters.
    @param pin the data pin of the ultrasonic sensor the perform an object detection
    @return the distance of an object in centimeters
*/
int getUltrasonicDistance(int pin)
{
  // Sent an ultrasonic tone
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pin, LOW);
  
  // Listen for the ultrasonic tone echo
  pinMode(pin, INPUT);
  int duration = pulseIn(pin, HIGH);
  //inches = duration / 74 / 2;
  return duration / 29 / 2;  // calculate and return the distance of an object
}

/** This function is executed when interrupt 1 is triggered. This handles commands sent to the Arduino
    when the trigger pin is set high.
*/
void handleCommand()
{
  byte command = getCommand();  // Get the command that was sent to the Arduino
  
  // Print the command to the serial monitor for debugging
  Serial.print("Command: ");
  Serial.println(command);
  
  // Determine which command was received
  switch(command)
  {
    case STOP:  // Stop the robot's wheels from moving
      stop();
      break;
    case MOVE_FORWARD:  // turn the wheels forward
      moveForward(0);
      break;
    case MOVE_BACKWARDS:  // turn the wheels backwards
      moveBackwards(0);
      break;
    case TURN_LEFT:  // rotate the robot counter clockwise
      rotateLeft();
      break;
    case TURN_RIGHT:  // rotate the robot clockwise
      rotateRight();
      break;
    case LOOK_LEFT:  // turn the camera to the left
      // Make sure the camera doesn't turn past it's max and min angle
      if(currentYawAngle + cameraSpeed >= MAX_YAW)
        currentYawAngle = MAX_YAW;
      else
        currentYawAngle += cameraSpeed;
      cameraYaw.write(currentYawAngle);
      delay(50);
      break;
    case LOOK_RIGHT:  // turn the camera to the right
      // Make sure the camera doesn't turn past it's max and min angle
      if(currentYawAngle - cameraSpeed <= MIN_YAW)
        currentYawAngle = MIN_YAW;
      else
        currentYawAngle -= cameraSpeed;
      cameraYaw.write(currentYawAngle);
      delay(50);
      break;
    case LOOK_UP:  // turn the camera up
      // Make sure the camera doesn't turn past it's max and min angle
      if(currentPitchAngle + cameraSpeed >= MAX_PITCH)
        currentPitchAngle = MAX_PITCH;
      else
        currentPitchAngle += cameraSpeed;
      cameraPitch.write(currentPitchAngle);
      delay(50);
      break;
    case LOOK_DOWN:  // turn the camera down
      // Make sure the camera doesn't turn past it's max and min angle
      if(currentPitchAngle - cameraSpeed <= MIN_PITCH)
        currentPitchAngle = MIN_PITCH;
      else
        currentPitchAngle -= cameraSpeed;
      cameraPitch.write(currentPitchAngle);
      delay(50);
      break;
    case START_FIRING:  // start firing the laser
      isFiring = true;
      break;
    case STOP_FIRING:  // stop firing the laser
      isFiring = false;
      break;
    case CENTER_CAMERA:  // move the camera to its center position
      currentPitchAngle = PITCH_CENTER_ANGLE;
      currentYawAngle = YAW_CENTER_ANGLE;
      cameraPitch.write(currentPitchAngle);
      cameraYaw.write(currentYawAngle);
      break;
  }
}

/** This function gets the command sent to the Arduino by reading the data pins as binary digits
    and converting the binary value to decimal. If a data pin is high, it is considered a '1'. If
    a data pin is low, it is considered a '0'.
    @return the decimal value of the command
*/
byte getCommand()
{
  byte bits[4];  // stores the data bit values
  byte command = 0;  // stores the decimal value to return
  
  bits[0] = digitalRead(pinDataBit0);  // gets the first bit value
  bits[1] = digitalRead(pinDataBit1);  // gets the second bit value
  bits[2] = digitalRead(pinDataBit2);  // gets the third bit value
  bits[3] = digitalRead(pinDataBit3);  // gets the fourth bit value
  
  // Calculate the decimal value
  command = bits[0];
  command += bits[1] * 2;
  command += bits[2] * 4;
  command += bits[3] * 8;
  
  return command;
}

/** Stop the right and left wheel servos */
void stop()
{
  rightWheel.write(WHEEL_STOP);
  leftWheel.write(WHEEL_STOP);
}

/** Rotate the robot in place to the left */
void rotateLeft()
{
  rightWheel.write(RIGHT_WHEEL_FORWARD);
  leftWheel.write(LEFT_WHEEL_BACKWARDS);
}

/** Rotate the robot in place to the right */
void rotateRight()
{
  rightWheel.write(RIGHT_WHEEL_BACKWARDS);
  leftWheel.write(LEFT_WHEEL_FORWARD);
}

/** Moves the robot forward. 
    @param arc arc is a value from -1.0 to 1.0 that determines the amount of turn to the
    left or right. 0 is straight forward. -1 is a sharp turn left. 1 is a sharp turn right.
*/
void moveForward(float arc)
{
  float lSpeed = WHEEL_STOP * speedScale;  // stores the speed of the left wheel
  float rSpeed = WHEEL_STOP * speedScale;  // stores the speed of the right wheel
  
  // if arc is 0, move the robot straight
  if(arc == 0)
  {
    rightWheel.write(WHEEL_STOP - rSpeed);
    leftWheel.write(WHEEL_STOP + lSpeed);
  }
  // if arc is positive, arc the robot's path to the right
  else if(arc > 0)
  {
    // Correct the value of arc if it is larger than the max value
    if(arc > 1)
      arc = 1;
    
    rSpeed = (float)rSpeed * arc;  // calculate the new speed of the right wheel based on arc
    rightWheel.write(round(rSpeed));
    leftWheel.write(WHEEL_STOP + lSpeed);
  }
  // if arc is negative, arc the robot's path to the left
  else if(arc < 0)
  {
    // Correct the value of arc if it is smaller than the min value
    if(arc < -1)
      arc = -1;
    
    lSpeed = lSpeed * arc;  // calculate the new speed of the left wheel based on arc
    rightWheel.write(WHEEL_STOP - rSpeed);
    leftWheel.write(round(180 + lSpeed));
  } 
}

/** Move the robot backwards.
    @param arc arc is a value from -1.0 to 1.0 that determines
    the amount of turn to the left or right. 0 is straight backwards. -1 is a sharp 
    turn left. 1 is a sharp turn right.
*/
void moveBackwards(float arc)
{
  float lSpeed = WHEEL_STOP * speedScale;  // stores the speed of the left wheel
  float rSpeed = WHEEL_STOP * speedScale;  // stores the speed of the right wheel
  
  // if arc is 0, move the robot straight
  if(arc == 0)
  {
    rightWheel.write(WHEEL_STOP + rSpeed);
    leftWheel.write(WHEEL_STOP - lSpeed);
  }
  
  // if arc is positive, arc the robot's path to the right
  else if(arc > 0)
  {
    // Correct the value of arc if it is larger than the max value
    if(arc > 1)
      arc = 1;
    
    rSpeed = (float)rSpeed * arc;  // calculate the new speed of the right wheel based on arc
    rightWheel.write(round(180 - rSpeed));
    leftWheel.write(WHEEL_STOP - lSpeed);
  }
  
  // if arc is negative, arc the robot's path to the left
  else if(arc < 0)
  {
    // Correct the value of arc if it is smaller than the min value
    if(arc < -1)
      arc = -1;
    
    lSpeed = (float)lSpeed * arc;  // calculate the new speed of the left wheel based on arc
    rightWheel.write(WHEEL_STOP + rSpeed);
    leftWheel.write(round(-lSpeed));
  } 
}
