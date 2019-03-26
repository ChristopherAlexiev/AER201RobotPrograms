/* i2c components based off of sample code for i2c from Michael Ding and Tyler Gamvrelis
 */

///////INCLUDE STATEMENTS////////////
////////////////////////////////////
#include <Wire.h>
#include <Servo.h>

///////PIN ASSIGNMENTS//////////////
////////////////////////////////////
//note that D3 is sketchy
/*
const int trigPin = 3;
const int echoPin = 12;
const int snapActionPin = 2;
const int electricBumpPin = 13;
const int cranePusherServoPin = 9;
const int linearServoPin = 6;
const int rotatorOnePin = 10;
const int rotatorTwoPin = 11;
*/

const int stepperPinOne = 2;
const int stepperPinTwo = 7;
const int stepperPinThree = 8;
const int stepperPinFour = 13;
const int trigPin = 3;
const int echoPin = 12;
const int snapActionPin = A3;
const int electricBumpPin = A2;
const int cranePusherServoPin = 9;
const int linearServoPin = 6;
const int rotatorOnePin = 10;
const int rotatorTwoPin = 11;

///////GLOBALS/////////////////////
//////////////////////////////////
//various constants
const int rotatorUpValueOne = 10;//maximum value for tire rotator servo
const int rotatorDownValueOne = 100;//minimum value for tire rotator servo 
const int rotatorUpValueTwo = 100;//maximum value for tire rotator servo
const int rotatorDownValueTwo = 10;//minimum value for tire rotator servo 
const int craneStopSpeed = 95;
const int linearStopSpeed = 95;
//need to add for rotator 1
const int ITwoCAddress = 8;

//moving ultrasonic avg
const int lengthAvg = 25; // note this used to be int const
long movingAvgArr[lengthAvg];
long movingAvgSum = 0;
int currentAvgArrayIndex = 0;
long movingAvg;

//servo objects
Servo cranePusherServo; //create crane pushing servo object
Servo linearServo;
Servo rotatorOne;
Servo rotatorTwo;

//volatile variables changed in interrupt functions
volatile bool send_to_pic = false;
volatile uint8_t incomingByte;
volatile bool doTireDrop;
volatile int snapSwitchTriggered = 0;
volatile int finishedYet = false;
volatile int tireDropRequested = false;
volatile int operationStarted = false;

//////INIT AND SETUP FUNCTIONS////////////////
///////////////////////////////////
void initMovingAvg(){
  //initialize avg array
  for (int i = 0; i < lengthAvg; i++ ) {
    movingAvgArr[i] = 200; /* set element at location i to i + 100 */
  }
  movingAvg = 200;
  movingAvgSum = 200*lengthAvg;
  return;
}



void setup(){
    //I2C setup
    Wire.begin(ITwoCAddress); // Join I2C bus with address 8
    Wire.onReceive(receiveEvent); // Called when this slave device receives a data transmission from master
    Wire.onRequest(requestEvent); // Called when master requests data from this slave device

    //attach servos to their ports
    cranePusherServo.attach(cranePusherServoPin);
    linearServo.attach(linearServoPin);
    rotatorOne.attach(rotatorOnePin);
    rotatorTwo.attach(rotatorTwoPin);
    
  
    // Open serial port to PC (hardware UART)
    Serial.begin(9600);      

    //ultrasonic setup
    pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
    pinMode(echoPin, INPUT); // Sets the echoPin as an Input

    //more pin setup
    pinMode(snapActionPin, INPUT);
    pinMode(electricBumpPin, INPUT);

    
}



/////////HELPER FUNCTIONS///////////////
////////////////////////////////////////

//get the ultrasonic distance reading from the sensor
long ultrasonicDistance(){
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    long duration = pulseIn(echoPin, HIGH, 4000);
    // Calculating the distance
    long distance= duration*0.034/2;
    // Prints the distance on the Serial Monitor

    return distance;
}

//change the movingAvg of the ultrasonic sensors value based on the current ultrasonic sensor reading
void changeUltrasonicMovingAvg(){
    long distance = 0;
    distance = ultrasonicDistance();
    movingAvgSum -= movingAvgArr[currentAvgArrayIndex];
    movingAvgSum += distance;
    movingAvgArr[currentAvgArrayIndex] = distance;
    movingAvg = movingAvgSum/lengthAvg;
    currentAvgArrayIndex ++;
    if (currentAvgArrayIndex == lengthAvg){
      currentAvgArrayIndex = 0;
    }
    return;
}


//lower flapper two. this is a blocking function
void lowerFlapperTwo(){//time in ms
    for (int pos = rotatorUpValueTwo; pos > rotatorDownValueTwo; pos -= 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      rotatorTwo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    return;
  /*
  unsigned long startTime = millis();

  unsigned long posPerTime = (rotatorDownValue - rotatorUpValue)/milliseconds;=
  \
  unsigned long pos = 130;
  for(milliseconds; milliseconds > 0 ; milliseconds --){
    pos += posPerTime
    rotatorTwo.write(pos);
    delay(1);
  }
  return;
  */
}
//raise flapper two. this is a blocking function
void raiseFlapperTwo(){//time in ms
    for (int pos = rotatorDownValueTwo; pos < rotatorUpValueTwo; pos += 2) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      rotatorTwo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
    }
    return;
}


//lower flapper one this is a blocking function
void lowerFlapperOne(){//time in ms
    for (int pos = rotatorUpValueOne; pos < rotatorDownValueOne; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      rotatorOne.write(pos);              // tell servo to go to position in variable 'pos'
      delay(20);                       // waits 15ms for the servo to reach the position
    }
    return;
 
}
//raise flapper one this is a blocking function
void raiseFlapperOne(){//time in ms
    for (int pos = rotatorDownValueOne; pos > rotatorUpValueOne; pos -= 2) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      rotatorOne.write(pos);              // tell servo to go to position in variable 'pos'
      delay(20);                       // waits 15ms for the servo to reach the position
    }
    return;
}

////////////////MAIN LOOP///////////////////
///////////////////////////////////////////

//note that the order of the flappers visited is 3,2,4,1 (from the starboard to port side of the robot is 1,2,3,4)
//flapperCase shows the flapper that must currently being accessed
//clampLoc shows the location of the clamp in terms of the flapper it is currently located at
int flapperCase = 3;//the first flapper to visit
int clampLoc = 2;
unsigned long startTime = 0;
unsigned long startTimeTwo = 0;

void loop(){

  while(!operationStarted){
    rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
    rotatorOne.write(rotatorUpValueOne);
    cranePusherServo.write(craneStopSpeed);//keep crane still

    cranePusherServo.write(95);
    //initMovingAvg();
    while (digitalRead(snapActionPin) == HIGH && !operationStarted){
    }  
    delay(50);
    
    cranePusherServo.write(0);
    while (digitalRead(snapActionPin) == LOW && !operationStarted){
    }
    delay(50);
    
    cranePusherServo.write(95);
    //initMovingAvg();
    while (digitalRead(snapActionPin) == HIGH && !operationStarted){

    } 
    delay(50); 
    cranePusherServo.write(180);
    while (digitalRead(snapActionPin) == LOW && !operationStarted){

    }       
    delay(50); 
    
  }
  
  
  if (flapperCase == 3 || flapperCase == 4){//the following cases are for if the tire is to be picked up from the 
    
    //START OF TIRE DROP PROCEDURE///////////////////////
 
  //init the servo motors
    rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
    rotatorOne.write(rotatorUpValueOne);
    cranePusherServo.write(craneStopSpeed);//keep crane still
  //wait for snap button
    /*while (digitalRead(snapActionPin) == HIGH){//wait for the snap action pin before crane moves

    }  
    delay(50);*/

    
    linearServo.write(0);//open clamp
    startTime = millis();
   
    
    
    
    //go to electric bump and stop then wait until enough time has passed to open clamp then lower tire
    cranePusherServo.write(180);//move to electric bump
    
    while (digitalRead(electricBumpPin) == LOW){
      if (millis()-startTime > 14000){
        linearServo.write(linearStopSpeed);
      }
    }
    if (flapperCase == 4){
      startTimeTwo = millis();
      while (millis()-startTimeTwo < 1000){
        
      }
      while (digitalRead(electricBumpPin) == LOW){
      if (millis()-startTime > 14000){
        linearServo.write(linearStopSpeed);
      }
    }
    }
    cranePusherServo.write(craneStopSpeed);

    while(millis()-startTime <= 14000){
    //wait for the time taken by the clamp to open  
    }
    linearServo.write(linearStopSpeed);
    
    lowerFlapperTwo(); //lower the flapper

    
    linearServo.write(180);//close the clamp
    delay(14000);

    //move in the other direction away from the flapper all the way to the outermost point
    cranePusherServo.write(0);//move clamp right until ebump farthest ebump pin
    delay(1000); //pass over the current bump pin
    while (digitalRead(electricBumpPin) == LOW){
    }
    
    delay(1000);//pass over the non-outermost ebump
    if (flapperCase ==4){
          while (digitalRead(electricBumpPin) == LOW){
          }
    
          delay(1000);//pass over the non-outermost ebump

    }

    cranePusherServo.write(0);  
    while (digitalRead(electricBumpPin) == LOW){
    }
    cranePusherServo.write(craneStopSpeed);//stop the clamp at the outermost ebump pin
     rotatorTwo.write(rotatorUpValueTwo);//non-blocking raising of the rotatorTwo
  //wait for the tire drop request////////
  while (!tireDropRequested){//wait for pic to send i2c tire drop request

  }
  tireDropRequested = false;//clear tire drop request
    
    //move the servo until the ultrasonic sensor sees the pole
    cranePusherServo.write(130);
    delay(2000);//pass over the non-outermost ebump and away from wall
    
    initMovingAvg();
    while (movingAvg>16){//move until the ultrasonic sensor sees the pole
      changeUltrasonicMovingAvg();
      Serial.println(movingAvg);
    }
    cranePusherServo.write(craneStopSpeed);

    //drop tire by opening clamp
    linearServo.write(0);
    delay(5000);
    
    
  } else {//if flapper is 1 or 2
    
    //START OF TIRE DROP PROCEDURE///////////////////////
 
  //init the servo motors
    rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
    rotatorOne.write(rotatorUpValueOne);
    cranePusherServo.write(craneStopSpeed);//keep crane still
    
  //wait for snap button
    while (digitalRead(snapActionPin) == HIGH){//wait for the snap action pin before crane moves

    }  
    delay(50);

    
    linearServo.write(0);//open clamp
    startTime = millis();
   
    
    
    
    //go to electric bump and stop then wait until enough time has passed to open clamp then lower tire
    cranePusherServo.write(0);//move to electric bump
    
    while (digitalRead(electricBumpPin) == LOW){
      if (millis()-startTime > 14000){
        linearServo.write(linearStopSpeed);
      }
    }
    if (flapperCase == 1){
      startTimeTwo = millis();
      while (millis()-startTimeTwo < 1000){
        
      }
      while (digitalRead(electricBumpPin) == LOW){
      if (millis()-startTime > 14000){
        linearServo.write(linearStopSpeed);
      }
    }
    }
    cranePusherServo.write(craneStopSpeed);

    while(millis()-startTime <= 14000){
    //wait for the time taken by the clamp to open  
    }
    linearServo.write(linearStopSpeed);
    
    lowerFlapperOne(); //lower the flapper

    
    linearServo.write(180);//close the clamp
    delay(14000);

    //move in the other direction away from the flapper all the way to the outermost point
    cranePusherServo.write(180);//move clamp until ebump farthest ebump pin
    delay(1000); //pass over the current bump pin
    while (digitalRead(electricBumpPin) == LOW){
    }
    
    delay(1000);//pass over the non-outermost ebump
    if (flapperCase ==1){
          while (digitalRead(electricBumpPin) == LOW){
          }
    
          delay(1000);//pass over the non-outermost ebump

    }

    cranePusherServo.write(180);  
    while (digitalRead(electricBumpPin) == LOW){
    }
    cranePusherServo.write(craneStopSpeed);//stop the clamp at the outermost ebump pin
     rotatorTwo.write(rotatorUpValueTwo);//non-blocking raising of the rotatorTwo
    
    //move the servo until the ultrasonic sensor sees the pole
    cranePusherServo.write(60);
    //delay(2000);//pass over the non-outermost ebump and away from wall
    
    initMovingAvg();
    while (movingAvg>16){//move until the ultrasonic sensor sees the pole
      changeUltrasonicMovingAvg();
      Serial.println(movingAvg);
    }
    cranePusherServo.write(craneStopSpeed);

    //drop tire by opening clamp
    linearServo.write(0);
    delay(5000);
        
  }
  linearServo.write(180);
  delay(1000);
  finishedYet = true;
}

////////////////////I2C INTERRUPT FUNCTIONS/////////////
////////////////////////////////////////////////////////
/** @brief Callback for when the master transmits data */
void receiveEvent(void){
    /*
    static uint8_t buf[3] = {0};
    static uint8_t counter = 0;
    */
    uint8_t x = Wire.read(); // Receive byte
    Serial.println((char)x); // Print to serial output as char (ASCII representation)
    if (x == '1'){
       tireDropRequested = true;
       finishedYet = false;
    } else if (x == '2'){//this is to signify the start of an operation
      operationStarted = true;
    }
    /*
    buf[counter++] = x;
    counter = (counter == 3) ? 0 : counter;
    
    if(buf[0]=='A' && buf[1]=='A' && buf[2]=='A'){
        send_to_pic = true;
    }
    */
}

/** @brief Callback for when the master requests data */
void requestEvent(void){
    if(finishedYet){
      Wire.write('1');
      Serial.println("TRUE");
    }else{
      Wire.write('0');
    }
}

/*
//this is the interrupt handling function
interruptHandler(){
  if (the interrupt is the snap action switch){
    clear interrupt;
    snapSwitchTriggered = true;
  }
}
*/

/////////END////////////
////////////////////////
