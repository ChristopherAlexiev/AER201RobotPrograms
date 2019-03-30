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
const int rotatorUpValueOne = 30;//maximum value for tire rotator servo
const int rotatorDownValueOne = 115;//minimum value for tire rotator servo 
const int rotatorUpValueTwo = 100;//maximum value for tire rotator servo
const int rotatorDownValueTwo = 10;//minimum value for tire rotator servo 
const int clampCloseValue = 10;
const int clampOpenValue = 80;
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
volatile int doOp;//do an operation
volatile int flapperCase = 3;//this shows the current flapper being visited; to be updated after every tire drop; 3 the first flapper to visit
volatile int firstTimeThrough = true; // this is set to true for the start of every operation

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
//to be used in an operation; will exit if the operation is aborted
void lowerFlapperTwo(){//time in ms
    for (int pos = rotatorUpValueTwo; pos > rotatorDownValueTwo; pos -= 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      rotatorTwo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
      if (!doOp){
        break;
      }
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
//raise flapper two at a lower than normal speed. this is a blocking function
//to be used in an operation; will exit if the operation is aborted
void raiseFlapperTwo(){//time in ms
    for (int pos = rotatorDownValueTwo; pos < rotatorUpValueTwo; pos += 2) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      rotatorTwo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(15);                       // waits 15ms for the servo to reach the position
      if (!doOp){
        break;
      }
    }
    return;
}


//lower flapper one at a lower than normal speed this is a blocking function
//to be used in an operation; will exit if the operation is aborted
void lowerFlapperOne(){//time in ms
    for (int pos = rotatorUpValueOne; pos < rotatorDownValueOne; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      rotatorOne.write(pos);              // tell servo to go to position in variable 'pos'
      delay(20);                       // waits 15ms for the servo to reach the position
      if(!doOp){
        break;
      }
    }
    return;
 
}



//raise flapper one this is a blocking function
//to be used in an operation; will exit if the operation is aborted
void raiseFlapperOne(){//time in ms
    for (int pos = rotatorDownValueOne; pos > rotatorUpValueOne; pos -= 2) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      rotatorOne.write(pos);              // tell servo to go to position in variable 'pos'
      delay(20);                       // waits 15ms for the servo to reach the position
      if (!doOp){
        break;
      }
    }
    return;
}


//lower flapper one at a lower than normal speed this is a blocking function
//to be used in an operation; will exit if the operation is aborted
void openClamp(){//time in ms
    for (int pos = clampCloseValue; pos < clampOpenValue; pos += 1) { 
      // in steps of 1 degree
      linearServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(10);                       // waits 15ms for the servo to reach the position
      if(doOp){
        break;
      }
    }
    return;
 
}



//raise flapper one this is a blocking function
//to be used in an operation; will exit if the operation is aborted
void closeClamp(){//time in ms
    for (int pos = clampOpenValue; pos > clampCloseValue; pos -= 1) { 
      // in steps of 1 degree
      linearServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(0);                       // waits 15ms for the servo to reach the position
      if (doOp){
        break;
      }
    }
    return;
}




//delay function that allows the operation to end when the doOp variable becomes false
//to be used within an operation
//input is the delayLength in ms
void opDelay(unsigned long delayLength){
  unsigned long timerStart = millis();
  while (millis()-timerStart < delayLength && doOp){
  }
  return;
}

//delay function that allows the operation to end when the doOp variable becomes true
//to be used outside an operation
//input is the delayLength in ms
void nonOpDelay(unsigned long delayLength){
  unsigned long timerStart = millis();
  while (millis()-timerStart < delayLength && !doOp){
  }
  return;
}

////////////////MAIN LOOP///////////////////
///////////////////////////////////////////

//note that the order of the flappers visited is 3,2,4,1 (from the starboard to port side of the robot is 1,2,3,4)
//flapperCase shows the flapper that must currently being accessed
//clampLoc shows the location of the clamp in terms of the flapper it is currently located at

int clampLoc = 2;
unsigned long startTime = 0;
unsigned long startTimeTwo = 0;

bool onBump = false;
int clampLocNeeded = 0;



void loop(){
  if (doOp){
    Serial.println("doing OP");
    if (firstTimeThrough){
       Serial.println("first time through reset");
       resetClampPartTwo();
       firstTimeThrough = false;
    }
    if (tireDropRequested){
      Serial.println("tire drop requested");
      //reset tire drop variable
      tireDropRequested = false;

      //drop the tire
      dropTire();
      //tell PIC that tire drop is complete
      Serial.println("tire drop complete");
      finishedYet = true;
      Serial.println("doing part 1 reset");
      resetClampPartOne();
      Serial.println("done part 1 reset");
      Serial.println("starting part 2 reset");
      resetClampPartTwo();
      Serial.println("done part 2 reset");


    }
  } else {
    Serial.println("Standby");
    standbyMode();
  }
}


/////////////OPERATION FUNCTIONS////////
////////////////////////////////////////
void standbyMode(){
    rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
    rotatorOne.write(rotatorUpValueOne);
    linearServo.write(linearStopSpeed);
    cranePusherServo.write(craneStopSpeed);//keep crane still

    cranePusherServo.write(craneStopSpeed);
    //initMovingAvg();
    while (digitalRead(snapActionPin) == HIGH && !doOp){
    }  
    nonOpDelay(50);
    linearServo.write(180);
    
    cranePusherServo.write(0);
    while (digitalRead(snapActionPin) == LOW && !doOp){
    }
    nonOpDelay(50);
    linearServo.write(0);
    cranePusherServo.write(craneStopSpeed);
    //initMovingAvg();
    while (digitalRead(snapActionPin) == HIGH && !doOp){

    } 
    nonOpDelay(50); 
    cranePusherServo.write(180);
    while (digitalRead(snapActionPin) == LOW && !doOp){

    }       
    nonOpDelay(50); 
    
}

void standbyMode2(){
    rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
    rotatorOne.write(rotatorUpValueOne);
    //linearServo.write(linearStopSpeed);
    cranePusherServo.write(craneStopSpeed);//keep crane still
    
    openClamp();
    
    //initMovingAvg();
    while (digitalRead(snapActionPin) == HIGH && !doOp){
    }  
    nonOpDelay(50);

    closeClamp();
    while (digitalRead(snapActionPin) == LOW && !doOp){
    }
    nonOpDelay(50);
    
}

void resetClampPartOne(){
  linearServo.write(180);
  Serial.println("starting 6 second delay");
  opDelay(6000);//used to be 10000

  //position clamp for next tire
  if (flapperCase == 3){
    flapperCase = 2;
  } else if (flapperCase == 2){
    flapperCase = 4;
  } else if (flapperCase == 4){
    flapperCase = 1;
  } else {
    flapperCase = 3;
  }
  if (flapperCase == 4 || flapperCase == 3){
    clampLocNeeded = 2;
  } else {
    clampLocNeeded = 2;
  }
  Serial.print("next flapperCase: ");
  Serial.println(flapperCase);
  Serial.print("clamp location needed: ");
  Serial.println(clampLocNeeded);

  if (clampLocNeeded > clampLoc){
    while (clampLocNeeded > clampLoc && doOp){
          cranePusherServo.write(180);  
            while (digitalRead(electricBumpPin) == LOW && doOp){
            }
            while (digitalRead(electricBumpPin) == HIGH && doOp){
            }
            clampLoc++;
        
    }cranePusherServo.write(craneStopSpeed);
  } else {
        while (clampLocNeeded < clampLoc && doOp){
          cranePusherServo.write(0);  
            while (digitalRead(electricBumpPin) == LOW && doOp){
            }
            while (digitalRead(electricBumpPin) == HIGH && doOp){
            }
            clampLoc--;
        
    }cranePusherServo.write(craneStopSpeed);
  }
  return;
}

const int servoLength = 2000; //used to be 14000

void resetClampPartTwo(){
  if (flapperCase == 3 || flapperCase == 4){//the following cases are for if the tire is to be picked up from the 
    ///THIS ASSUMES IT STARTS IN POSITION 2
    //START OF TIRE DROP PROCEDURE///////////////////////
 
  //init the servo motors
    rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
    rotatorOne.write(rotatorUpValueOne);
    cranePusherServo.write(craneStopSpeed);//keep crane still
  //wait for snap button
    /*while (digitalRead(snapActionPin) == HIGH && doOp){//wait for the snap action pin before crane moves

    }  
    opDelay(50);*/

    
    linearServo.write(0);//open clamp
    startTime = millis();
   
    
    
    
    //go to electric bump and stop then wait until enough time has passed to open clamp then lower tire
    cranePusherServo.write(180);//move to electric bump
    
    while (digitalRead(electricBumpPin) == LOW && doOp){
      if (millis()-startTime > servoLength){
        linearServo.write(linearStopSpeed);
      }
    }
    
    if (flapperCase == 4){
      while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
        if (millis()-startTime > servoLength){
          linearServo.write(linearStopSpeed);
        }
      }
      //startTimeTwo = millis();//this loop no longer needed with more reliable non timed algorithm
      //while (millis()-startTimeTwo < 1000 && doOp){
        
      //}
      while (digitalRead(electricBumpPin) == LOW && doOp){
        if (millis()-startTime > servoLength){
          linearServo.write(linearStopSpeed);
        }
      }
    }
    while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
        if (millis()-startTime > servoLength){
          linearServo.write(linearStopSpeed);
        }
    }
    cranePusherServo.write(craneStopSpeed);

    while(millis()-startTime <= servoLength && doOp){
    //wait for the time taken by the clamp to open  
    }
    linearServo.write(linearStopSpeed);
    
    lowerFlapperTwo(); //lower the flapper

    
    linearServo.write(180);//close the clamp
    opDelay(servoLength);

    //move in the other direction away from the flapper all the way to the outermost point
    cranePusherServo.write(0);//move clamp right until ebump farthest ebump pin
    //opDelay(1000); //pass over the current bump pin SHOULDN'T BE NEEDED WITH NON TIMED ALGORITHM

    //pass over the nearest bump:
    while (digitalRead(electricBumpPin) == LOW && doOp){
              opDelay(50);
    }
    while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
      opDelay(50);
    }
    Serial.println("FOUND");
    while (digitalRead(electricBumpPin) == LOW && doOp){
              opDelay(50);
    }
    while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
      opDelay(50);
    }

    Serial.println("FOUND");
    //opDelay(1000);//pass over the non-outermost ebump SHOULDN'T BE NEEDED WITH NON TIMED ALGORITHM
    if (flapperCase ==4){
          while (digitalRead(electricBumpPin) == LOW && doOp){
                    opDelay(50);
          }
    
          //opDelay(1000);//pass over the non-outermost ebump
          while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
                opDelay(50);
          }

    }


    while (digitalRead(electricBumpPin) == LOW && doOp){//get to the outermost ebump pin
              opDelay(50);
    }
    while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
          opDelay(50);
    }
    
    Serial.println("FOUND");
    cranePusherServo.write(craneStopSpeed);//stop the clamp at the outermost ebump pin
    rotatorTwo.write(rotatorUpValueTwo);//non-blocking raising of the rotatorTwo
  
  } else {//if flapper is 1 or 2
    //THIS ASSUMES THE TIRE STARTS IN POSITION 2
    
    //START OF TIRE DROP PROCEDURE///////////////////////
 
  //init the servo motors
    rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
    rotatorOne.write(rotatorUpValueOne);
    cranePusherServo.write(craneStopSpeed);//keep crane still
    
  //wait for snap button
    /*while (digitalRead(snapActionPin) == HIGH && doOp){//wait for the snap action pin before crane moves

    }  
    opDelay(50);*/

    
    linearServo.write(0);//open clamp
    startTime = millis();
   
    
    
    
    //go to electric bump and stop then wait until enough time has passed to open clamp then lower tire
    cranePusherServo.write(0);//move to electric bump
    
    while (digitalRead(electricBumpPin) == LOW && doOp){
      if (millis()-startTime > servoLength){
        linearServo.write(linearStopSpeed);
      }
    }
    if (flapperCase == 1){
      //startTimeTwo = millis(); //no longer needed with non timed algorithm
      //while (millis()-startTimeTwo < 1000 && doOp){
        
      //}
      while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
        if (millis()-startTime > servoLength){
          linearServo.write(linearStopSpeed);
        }
      }
      while (digitalRead(electricBumpPin) == LOW && doOp){
      if (millis()-startTime > servoLength){
        linearServo.write(linearStopSpeed);
      }
      }
    }
    while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
        if (millis()-startTime > servoLength){
          linearServo.write(linearStopSpeed);
        }
    }
    cranePusherServo.write(craneStopSpeed);

    while(millis()-startTime <= servoLength && doOp){
    //wait for the time taken by the clamp to open  
    }
    linearServo.write(linearStopSpeed);
    
    lowerFlapperOne(); //lower the flapper

    
    linearServo.write(180);//close the clamp
    opDelay(servoLength);

    //move in the other direction away from the flapper all the way to the outermost point
    cranePusherServo.write(180);//move clamp until ebump farthest ebump pin

    while (digitalRead(electricBumpPin) == LOW && doOp){
    }
    while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
    }

    while (digitalRead(electricBumpPin) == LOW && doOp){
    }
    while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
    }
    while (digitalRead(electricBumpPin) == LOW && doOp){
    }
    while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
    }
    
    //opDelay(1000);//pass over the non-outermost ebump //no longer needed for non timed algorithm
    if (flapperCase == 1){
          while (digitalRead(electricBumpPin) == LOW && doOp){
          }
          while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
          }
          //opDelay(1000);//pass over the non-outermost ebump

    }
    //the crane should now be to the left of ebump 4
    /*cranePusherServo.write(180);  
    while (digitalRead(electricBumpPin) == LOW && doOp){
    }*/
    cranePusherServo.write(craneStopSpeed);//stop the clamp at the outermost ebump pin
     rotatorOne.write(rotatorUpValueOne);//non-blocking raising of the rotatorTwo
  }
  return;
}

void dropTire(){ 
  if (flapperCase == 3 || flapperCase == 4){//the following cases are for if the tire is to be picked up from the 

    //move the clamp to be to the left of the ebump pin 1, and label this as location 1
    cranePusherServo.write(180);
    //pass over the nearest bump:
    while (digitalRead(electricBumpPin) == LOW && doOp){
    }
    while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
    }
    clampLoc = 1;

  //opDelay(1000); //pass over the current bump pin SHOULDN'T BE NEEDED WITH NON TIMED ALGORITHM

    
    //move the servo until the ultrasonic sensor sees the pole
    cranePusherServo.write(130);
    //opDelay(1000);//drive away from wall for the ultrasonic sensor
    onBump = false;
    initMovingAvg();
    unsigned long startingTime =millis();
    while (movingAvg>16 && doOp){//move until the ultrasonic sensor sees the pole
      if (millis()-startingTime > 2000){//drive away from wall for the ultrasonic sensor
        changeUltrasonicMovingAvg();
      }
      //Serial.println(movingAvg);
      Serial.println(clampLoc);
      if (digitalRead(electricBumpPin) == HIGH && onBump == false){
            onBump = true;
      }
      if (digitalRead(electricBumpPin) == LOW && onBump == true){
             onBump = false;
             clampLoc ++;
      }
      opDelay(50);
    }
    Serial.println("pole found");
    cranePusherServo.write(craneStopSpeed);

    //drop tire by opening clamp
    linearServo.write(0);
    opDelay(5000);
    linearServo.write(linearStopSpeed);  
    
  } else {//if flapper is 1 or 2

    //move the clamp to be to the right of the ebump pin 4, and label this as location 3
    cranePusherServo.write(0);//move clamp to the right of ebump pin 4
    //pass over the nearest bump:
    while (digitalRead(electricBumpPin) == LOW && doOp){
    }
    while (digitalRead(electricBumpPin) == HIGH && doOp){//pass over bump to avoid the need for a less-reliable timed algorithm
    }
    clampLoc = 3;

    //move the servo until the ultrasonic sensor sees the pole
    cranePusherServo.write(60);
    //delay(2000);//pass over the non-outermost ebump and away from wall
    onBump = false;
    initMovingAvg();
    unsigned long startingTime =millis();
    while (movingAvg>16 && doOp){//move until the ultrasonic sensor sees the pole
      if (millis()-startingTime > 0){//drive away from wall for the ultrasonic sensor
        changeUltrasonicMovingAvg();
      }
      Serial.println(clampLoc);
      //Serial.println(movingAvg);
          if (digitalRead(electricBumpPin) == HIGH && onBump == false){
            onBump = true;
          }
          if (digitalRead(electricBumpPin) == LOW && onBump == true){
             onBump = false;
             clampLoc --;
          }
    }
    cranePusherServo.write(craneStopSpeed);

    //drop tire by opening clamp
    linearServo.write(0);
    opDelay(2000);//close clamp
    linearServo.write(linearStopSpeed);   
  }
  return;
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
    //Serial.println((char)x); // Print to serial output as char (ASCII representation)
    if (x == '1'){
       tireDropRequested = true;
       finishedYet = false;
    } else if (x == '2'){//this is to signify the start of an operation
      flapperCase = 3;//the first flapper in an operation is flapper 3
      firstTimeThrough = true;
      doOp = true;
    } else if (x == '3'){ // this is to signify the end of an operation... aka the operation is aborted or time is up
      doOp = false; 
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
      //Serial.println("TRUE");
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
