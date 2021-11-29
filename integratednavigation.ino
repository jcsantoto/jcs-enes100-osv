/*https://docs.google.com/document/d/1a-Q39kCzKcpODhkTnCgKBXocIx9LJcBp9rpjuLiAvHM/edit?usp=sharing original*/

/*From ENES100 Website*/
#include <Enes100.h>

/*Already in arduino*/
#include <Wire.h>

/*From https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/install-software*/
#include <Adafruit_MotorShield.h>

#include <math.h>

//Integrated Navigation v 2.1

/* Pin 0-CANNOT USE
 * Pin 1-CANNOT USE
 * Pin 2 - APC220 RX
 * Pin 3 - APC220 TX
 * Pin 4
 * Pin 5
 * Pin 6
 * Pin 7 - RangeFinder1
 * Pin 8 - RangeFinder1
 * Pin 9
 * Pin 10
 * Pin 11
 * Pin 12 - RangeFinder2
 * Pin 13 - RangeFinder2
 */

//Declares the pins used by the range finders.
#define trigPin1 7
#define trigPin2 8
#define echoPin1 13
#define echoPin2 12

//Creates the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

//Left Motors use Ports M1 and M4
Adafruit_DCMotor *myMotor = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(4);

//Right Motors use Ports M2 and M3
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(3);

void setup() {

//Rangefinders
 pinMode(trigPin1,OUTPUT);
 pinMode(trigPin2,OUTPUT);
 pinMode(echoPin1,INPUT);
 pinMode(echoPin2, INPUT);

//Starts up the Motor Shield
  AFMS.begin(); 


// Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor->setSpeed(70);
  myMotor1->setSpeed(70);
  myMotor2->setSpeed(70);
  myMotor3->setSpeed(70);

   //APCS220 Connection & Begin Mission
   //@Param Team Name, Marker # , RX, TX
  while(!Enes100.begin("TBD",DEBRIS,11,2,3)) {
    Enes100.println("Unable to connect to simulation");
    delay(300);
  }
  
    Enes100.println("Starting Navigation");

    while (!Enes100.updateLocation()) {
      Enes100.println("Unable to update Location");
    }

    Enes100.updateLocation();
    
    /* To start, orient the OSV towards the rocky terrain.
     *The forward direction will be at theta = 0
     */
    faceForward();

  }

  void loop() {

    //Move Forward
    myMotor->run(FORWARD);   myMotor3->run(FORWARD);
    myMotor1->run(BACKWARD);   myMotor2->run(BACKWARD);

    //Provides the current location of the OSV
    Enes100.updateLocation();
    float x = Enes100.location.x;
    float y = Enes100.location.y;
    float t = Enes100.location.theta;

    // Enes100.println("Xcoord:");
    // Enes100.print(x);

    Enes100.println("Ycoord");
    Enes100.print(y);

    // Enes100.println("Theta");
    // Enes100.print(t);



/*
 * When the OSV's X coordinate is near the Objective's X coordinate,
 * meaning that they nearly align along the y-axis, 
 * Turn towards the Objective and Move towards it.
 * 
 */
    if (Enes100.updateLocation() && abs(Enes100.destination.x - Enes100.location.x) < 0.3) {

      if (Enes100.location.y < Enes100.destination.y) {
        turnLeft(false);
        moveForward();
      } else {
        turnRight(false);
        moveForward();
      }

    }

    /*
     * while(atObstacle()){
     * 
     * myMotor->run(RELEASE);   myMotor3->run(RELEASE);       myMotor1->run(RELEASE);   myMotor2->run(RELEASE); //turnTowardMission();
     * Enes100.println("At mission site");
     * 
     * }
     */

    /*
     * If both range finders detect obstacles, back up slightly and
     * If the OSV is located in the upper half of the field, turn right,
     * Else turn left.
     * (This makes sure that the OSV is turning towards the middle instead of 
     * the edges of the arena)
     * 
     * Then move forward slightly and reorient to face forward
     * 
     */
    if (getRangeFinder1() < 0.2 && getRangeFinder2() < 0.2) {
    
      backUp();

      if (y < 1) {
        turnRight(true);
      } else {
        turnLeft(true);
      }

      moveForward();
      faceForward();
    }

/*
 * If ONLY the LEFT sensor detects an obstacle, meaning that the right side is clear,
 * then back up slightly, turn right, move forward, and then reorient to face forward.
 */
    else if (getRangeFinder1() < 0.2 && !(getRangeFinder1() < 0.2)) {
      // Enes100.println("Detect Obstacle");
      backUp();
      turnRight(true);
      moveForward();
      faceForward();
    }

/*
 * If ONLY the RIGHT sensor detects an obstacle, meaning that the left side is clear,
 * then back up slightly, turn left, move forward, and then reorient to face forward.
 */
    else if (!(getRangeFinder1() < 0.2) && getRangeFinder2() < 0.2) {
      // Enes100.println("Detect Obstacle");
      backUp();
      turnLeft(true);
      moveForward();
      faceForward();
    }


    /*
     * If the OSV happens to get stuck,
     * back up, turn right, move forward and reorient to face forward.
     */
    else if (isStationary(Enes100.location.x, Enes100.location.y,
        Enes100.location.theta)) {
    
      backUp();
      turnRight(true);
      moveForward();
      faceForward();
    }

    /*
     * If the OSV should hit the bottom wall, 
     * back up and reorient to face forward.
     * Note: This will only trigger if the OSV is facing the wall
     */
    if (y < 0.3 && t < -2) {
      backUp();
      faceForward();
      moveForward();

    }
    
     /*
     * If the OSV should hit the top wall, 
     * back up and reorient to face forward.
     * Note: This will only trigger if the OSV is facing the wall
     */
    else if (y > 1.85 && t > 2) {
      backUp();
      faceForward();
      moveForward();

    }


/*These are for if the OSV hits the Left or Right wall however
 * they do not currently work properly.
 * 
    // If OSV hits left wall, face forward and move
    if (x < 0.15) {
      // Enes100.println("OSV is at left wall");
      faceForward();
      moveForward();

    }

    // If OSV hits right wall, turn right and move
    else if (x > 3.85) {
      // Enes100.println("OSV is at right wall");
      turnRight(false);
      moveForward();

    }
*/


    // Move forward
    myMotor->run(FORWARD);   myMotor3->run(FORWARD);
    myMotor1->run(BACKWARD);   myMotor2->run(BACKWARD);

  }
  
////////////////////////
/*
 * FUNCTIONS
 */
 ////////////////////////
 
  //Moves the OSV forward for 1 seconds.
  void moveForward() {

    myMotor->run(FORWARD);   myMotor3->run(FORWARD);
    myMotor1->run(BACKWARD);   myMotor2->run(BACKWARD);
    delay(1000);
  }

  //Move the OSV to face forward (Theta = 0)
  void faceForward() {


//If the OSV is facing downwards(theta < 0)
    if (Enes100.location.theta < 0) {

//Then turn left, incrementally, until theta is positive.
      while ((Enes100.location.theta) < 0) {

        myMotor->run(BACKWARD);   myMotor3->run(BACKWARD);
        myMotor1->run(BACKWARD);   myMotor2->run(BACKWARD);
        delay(100);
        myMotor->run(RELEASE);   myMotor3->run(RELEASE);       myMotor1->run(RELEASE);   myMotor2->run(RELEASE);

        float x = Enes100.location.x;
        float y = Enes100.location.y;
        float t = Enes100.location.theta;

/*
 * If the OSV should happen to get stuck while turning(This happens often in the simulation),
 * Then we break out of the while loop. 
 * This is because the while loop will infinitely run since it never achieves theta > 0 if it is stuck.
 */
        if (isStationary(Enes100.location.x, Enes100.location.y,
            Enes100.location.theta)) {
          break;
        }
        while (!Enes100.updateLocation()) {
          Enes100.println("Unable to update location");
        }

      }
    }


/*The other case is if Theta > 0, meaning it faces upwards
 * This is the same process but turning right until theta < 0.
 */
    else {

      while ((Enes100.location.theta) > 0) {

        myMotor->run(FORWARD);   myMotor3->run(FORWARD);
        myMotor1->run(FORWARD);   myMotor2->run(FORWARD);
        delay(100);
        myMotor->run(RELEASE);   myMotor3->run(RELEASE);       myMotor1->run(RELEASE);   myMotor2->run(RELEASE);

        float x = Enes100.location.x;
        float y = Enes100.location.y;
        float t = Enes100.location.theta;

        if (isStationary(Enes100.location.x, Enes100.location.y,
            Enes100.location.theta)) {
          break;
        }
        while (!Enes100.updateLocation()) {
          Enes100.println("Unable to update location");
        }

      }

    }

  }

  /*
   * Turns the OSV to the Right.
   * @Param boolean obstacle 
   * This parameter represents whether the OSV is turning because it has encountered an obstacle.
   * 
   * The OSV will make turns in other cases that will not involve obstacle avoidance(e.g. Turning towards the objective)
   */
  void turnRight(boolean obstacle) {

//Turns the OSV right incrementaly until theta < pi/2 (basically facing south)
    while ((Enes100.location.theta) > -PI / 2.1) {

      myMotor->run(FORWARD);   myMotor3->run(FORWARD);
      myMotor1->run(FORWARD);   myMotor2->run(FORWARD);
      delay(100);
      myMotor->run(RELEASE);   myMotor3->run(RELEASE);       myMotor1->run(RELEASE);   myMotor2->run(RELEASE);

/*
 * However! , If the OSV sees a clear path while turning, 
 * Then we break the while loop and move forward.
 * 
 * This is so that the OSV does not always have to make a full 90deg turn to avoid the obstacle.
 * Note: This should only run in the event that it must avoid an obstacle meaning the parameter must be true.
 */
      if (obstacle && (getRangeFinder1() > 0.2
          && getRangeFinder2() > 0.2)) {
        break;
      }

//If the OSV should happen to get stuck while turning, break out the loop
      else if (isStationary(Enes100.location.x, Enes100.location.y,
          Enes100.location.theta)) {
        break;
      }

      while (!Enes100.updateLocation()) {
        Enes100.println("Unable to update location");
      }

    }

  }

  /*Turns the OSV to the left.
   * The process is identical to the turnRight() function.
   * Read turnRight() to understand how this function will work.
   */
  void turnLeft(boolean obstacle) {

    while ((Enes100.location.theta) < PI / 2.1) {

      myMotor->run(BACKWARD);   myMotor3->run(BACKWARD);
      myMotor1->run(BACKWARD);   myMotor2->run(BACKWARD);
      delay(100);
      myMotor->run(RELEASE);   myMotor3->run(RELEASE);       myMotor1->run(RELEASE);   myMotor2->run(RELEASE);

      if (obstacle && (getRangeFinder1() > 0.2
          && getRangeFinder2() > 0.2)) {
        break;
      }

      else if (isStationary(Enes100.location.x, Enes100.location.y,
          Enes100.location.theta)) {
        break;
      }

      while (!Enes100.updateLocation()) {
        Enes100.println("Unable to update location");
      }

    }

  }

  /*Checks if the OSV is stationary/stuck
   * @Param previous x, y and theta values
   * 
   * This methods takes in the previous x, y, & theta values 
   * and compares them to current values. (The values are 200 milleseconds apart)
   * 
   */
  boolean isStationary(float x1, float y1, float t1) {

    Enes100.updateLocation();
    delay(200);
    Enes100.updateLocation();

    float x2 = Enes100.location.x;
    float y2 = Enes100.location.y;
    float t2 = Enes100.location.theta;

/*
 * These are the ideal conditions for the simulator.
 * Realistically, you would check if x1 and x2 , etc. are within a certain range of eachother.
 */
    if (x1 == x2 && y1 == y2 && t1 == t2) {
      Enes100.println("Stuck");
      return true;
    }

    else {
      return false;
    }

  }

  //Backs up the OSV for a very short duration
  void backUp() {

    myMotor->run(BACKWARD);   myMotor3->run(BACKWARD);
    myMotor1->run(FORWARD);   myMotor2->run(FORWARD);
    delay(150);

  }

/*
 * This value calcualtes the angle at which the OSV  must turn in order
 * to be facing towards the objective.
 */
  void turnTowardMission() {

    Enes100.updateLocation();

    float x1 = Enes100.location.x;
    float y1 = Enes100.location.y;

    float x2 = Enes100.destination.x;
    float y2 = Enes100.destination.y;

    float theta2 = atan2((y2 - y1), (x2 - x1));

    while ((abs(Enes100.location.theta) - abs(theta2)) > 0.05) {

      Enes100.updateLocation();
      myMotor->run(BACKWARD);   myMotor3->run(BACKWARD);
      myMotor1->run(BACKWARD);   myMotor2->run(BACKWARD);

      while (!Enes100.updateLocation()) {
        Enes100.println("Unable to update location");
      }

    }
  }


float getRangeFinder1(){

  int duration1;
  int distance1;


  digitalWrite(trigPin1,LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1,LOW);


  duration1 = pulseIn(echoPin1,HIGH);
  distance1 = (duration1/2)*0.0344;
  
}

float getRangeFinder2(){
  
  int duration2;
int distance2;

  digitalWrite(trigPin2,LOW);
    delayMicroseconds(2);
  digitalWrite(trigPin2,HIGH);
   delayMicroseconds(10);
    digitalWrite(trigPin2,LOW);
    
  duration2 = pulseIn(echoPin2,HIGH);
  distance2 = (duration2/2)*0.0344;
  
}
  

