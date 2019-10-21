/* i2c components based off of sample code for i2c from Michael Ding and Tyler Gamvrelis
 */

///////INCLUDE STATEMENTS////////////
////////////////////////////////////
#include <Wire.h>
#include <Servo.h>
#include <Stepper.h>


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
const int ebumpFourPin = A0;
const int ebumpThreePin = A2;
const int ebumpTwoPin = A1;
const int ebumpOnePin = A3;
const int cranePusherServoPin = 9;
const int linearServoPin = 6;
const int rotatorOnePin = 10;
const int rotatorTwoPin = 11;

///////GLOBALS/////////////////////
//////////////////////////////////
//various constants
const int rotatorUpValueOne = 25;//maximum value for tire rotator servo                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         [[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[
const int rotatorDownValueOne = 115;//minimum value for tire rotator servo 
const int rotatorUpValueTwo = 100;//maximum value for tire rotator servo
const int rotatorDownValueTwo = 10;//minimum value for tire rotator servo 
const int clampCloseValue = 0;
const int clampOpenValue = 100;
const int craneStopSpeed = 93;
const int linearStopSpeed = 95;
const unsigned long opDelayAmount = 20;
const int ITwoCAddress = 8;
const int stepsPerTire = 700;
const int stepsPerRevolution = 200;
const int stepSpeed = 50;

//note that these aren't the only speeds used; there are some magic numbers particularly for the ultrasonic sensor
const int craneFastPort = 35;
const int craneMediumPort = 60;
const int craneFastStarboard = 145;
const int craneMediumStarboard = 130;
const int craneSlowStarboard = 110;
const int craneSlowPort = 75;

//ultrasonic distance constants

const long int minPoleHeight = 14ll;
const long int maxPoleHeight = 18ll;
const long int minGroundHeight = 24ll;
const long int maxGroundHeight = 28ll;


//moving ultrasonic avg
const int lengthAvg = 20; // note this used to be int const
long movingAvgArr[lengthAvg];
long movingAvgSum = 0;
int currentAvgArrayIndex = 0;
long movingAvg;

//Motor objects
Servo cranePusherServo; //create crane pushing servo object
Servo linearServo;
Servo rotatorOne;
Servo rotatorTwo;
Stepper steppy(stepsPerRevolution, stepperPinOne, stepperPinTwo, stepperPinThree, stepperPinFour);

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
volatile bool standbyClampMove = false;
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
    //stepper setup
    steppy.setSpeed(100);
  
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
    pinMode(ebumpFourPin, INPUT);
    pinMode(ebumpThreePin, INPUT);
    pinMode(ebumpTwoPin, INPUT);
    pinMode(ebumpOnePin, INPUT);
    
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

    //if the distance measured is an unrealistic number then do not change the moving average
    if (distance < minPoleHeight || (distance > maxPoleHeight && distance < minGroundHeight) || distance > maxGroundHeight){
      return;
    }

    //change the moving avg based on the distance measured
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
      delay(35);                       // waits 15ms for the servo to reach the position
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
      delay(25);                       // waits 15ms for the servo to reach the position
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
      delay(35);                       // waits 15ms for the servo to reach the position
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


//open the clamp; this is a blocking function
//to be used in an operation; will exit if the operation is aborted
void closeClamp(){//time in ms
    for (int pos = clampOpenValue; pos > clampCloseValue; pos -= 1) { 
      // in steps of 1 degree
      linearServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(3);                       // waits 15ms for the servo to reach the position
      if(!doOp){
        break;
      }
    }
    return;
 
}



//close the clamp; this is a blocking function
//to be used in an operation; will exit if the operation is aborted
void openClamp(){//time in ms
    for (int pos = clampCloseValue; pos < clampOpenValue; pos += 1) { 
      // in steps of 1 degree
      linearServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(3);                       // waits 15ms for the servo to reach the position
      if (!doOp){
        break;
      }
    }
    return;
}

//open the clamp; this is a blocking function
//to be used outside an operation; will exit if the operation is aborted
void nonOpCloseClamp(){//time in ms
    for (int pos = clampOpenValue; pos > clampCloseValue; pos -= 1) { 
      // in steps of 1 degree
      linearServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(3);                       // waits 15ms for the servo to reach the position
      if(doOp){
        break;
      }
    }
    return;
 
}



//close the clamp; this is a blocking function
//to be used outside an operation; will exit if the operation is aborted
void nonOpOpenClamp(){//time in ms
    for (int pos = clampCloseValue; pos < clampOpenValue; pos += 1) { 
      // in steps of 1 degree
      linearServo.write(pos);              // tell servo to go to position in variable 'pos'
      delay(3);                       // waits 15ms for the servo to reach the position
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

bool inRangeInclusive(long number, int minimum, int maximum){
  if (number >= (long int) minimum && number <= (long int)maximum){
    return true;
  } else {
    return false;
  }
}

////////////////MAIN LOOP///////////////////
///////////////////////////////////////////

//note that the order of the flappers visited is 3,2,4,1 (from the starboard to port side of the robot is 1,2,3,4)
//flapperCase shows the flapper that must currently being accessed
//clampLoc shows the location of the clamp in terms of the flapper it is currently located at

int clampLoc = 2;
unsigned long startTime = 0;
unsigned long startTimeTwo = 0;


int clampLocNeeded = 0;



void loop(){
  
  
  initMovingAvg();
  
  //openClamp();
  while (true){
  
    if (doOp){
      Serial.println("doing OP");
      if (firstTimeThrough){
         Serial.println("first time through reset");
         clampLoc = 2;//the clamp should be moved by hand to here before the start of the operation
         flapperCase = 1;//the reset clamp functions pre-increment this value to the actual starting flappercase of 3
         openClamp();
         resetClampPartOne();//take the clamp from the start position (in location)
         resetClampPartTwo();
         firstTimeThrough = false;
      }
      if (tireDropRequested){
        Serial.println("tire drop requested");
        Serial.println("flapperCase: " + flapperCase);
        //reset tire drop variable
        tireDropRequested = false;
        if (flapperCase == 1){//if this is the last flapper then move all the tires over
          Serial.println("STEPPY");
          steppy.step(-stepsPerTire);
          digitalWrite(stepperPinOne, LOW);
          digitalWrite(stepperPinTwo, LOW);
          digitalWrite(stepperPinThree, LOW);
          digitalWrite(stepperPinFour, LOW);
          Serial.println("DONE STEPPY");
          //steppy.step(-stepsPerTire);
          //for (int i = 0; i<10; i++){
            //steppy.step(-1);
            //opDelay(10);
          //}
        }
        //drop the tire
        dropTire();
        //tell PIC that tire drop is complete
        
        Serial.println("tire drop complete");
        Serial.println("Clamp location:");
        Serial.println(clampLoc);
        //opDelay(5000);
        finishedYet = true;
 
        Serial.println("doing part 1 reset");

        resetClampPartOne();

        Serial.println("done part 1 reset");

        Serial.println("starting part 2 reset");
        resetClampPartTwo();
        
        Serial.println("done part 2 reset");


      }
    } else {
      //Serial.println("Standby");
      //rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
      //rotatorOne.write(rotatorUpValueOne);
      //cranePusherServo.write(180);
      standbyModeInterruptUltrasonicTest();
      //standbyModeInterrupt();
    }

  }
 
  //linearServo.write(0);
  //cranePusherServo.write(craneStopSpeed);
  //rotatorTwo.write(rotatorUpValueTwo);
  //rotatorOne.write(40);
  //delay(3000);
}


/////////////OPERATION FUNCTIONS////////
////////////////////////////////////////

//old normal standby mode using the snap switch
void standbyMode(){
    rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
    rotatorOne.write(rotatorUpValueOne);
    Serial.println("OFF");
    //cranePusherServo.write(craneStopSpeed);//keep crane still
    
    cranePusherServo.write(craneStopSpeed);
    //initMovingAvg();
    while (digitalRead(snapActionPin) == LOW && !doOp){
    }  
    nonOpDelay(50);
    
    //linearServo.write(180);
    Serial.println("RIGHT");    
    cranePusherServo.write(0);
    while (digitalRead(snapActionPin) == HIGH && !doOp){
    }
    nonOpDelay(50);
    //linearServo.write(craneStopSpeed);
    cranePusherServo.write(craneStopSpeed);
    //initMovingAvg();
    Serial.println("OFF");    
    while (digitalRead(snapActionPin) == LOW && !doOp){

    } 
    nonOpDelay(50); 
    cranePusherServo.write(180);
        Serial.println("LEFT");    
    while (digitalRead(snapActionPin) == HIGH && !doOp){

    }       
    nonOpDelay(50); 
    cranePusherServo.write(craneStopSpeed);
    return;
    
}

//the standby mode using the A button on the devbugger keypad
void standbyModeInterrupt(){
    rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
    rotatorOne.write(rotatorUpValueOne);
    Serial.println("OFF");
    //cranePusherServo.write(craneStopSpeed);//keep crane still
    
    cranePusherServo.write(craneStopSpeed);
    //initMovingAvg();
    while (!standbyClampMove && !doOp){
    }  
    nonOpDelay(50);
    
    //linearServo.write(180);
    Serial.println("RIGHT");    
    cranePusherServo.write(60);
    while (standbyClampMove && !doOp){
    }
    nonOpDelay(50);
    //linearServo.write(craneStopSpeed);
    cranePusherServo.write(craneStopSpeed);
    //initMovingAvg();
    Serial.println("OFF");    
    while (!standbyClampMove && !doOp){

    } 
    nonOpDelay(50); 
    cranePusherServo.write(120);
    Serial.println("LEFT");    
    while (standbyClampMove && !doOp){

    }       
    nonOpDelay(50); 
    cranePusherServo.write(craneStopSpeed);
    return;
}


//the standby mode using the A button on the devbugger keypad
void standbyModeInterruptUltrasonicTest(){
    rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
    rotatorOne.write(rotatorUpValueOne);
    Serial.println("OFF");
    //cranePusherServo.write(craneStopSpeed);//keep crane still
    
    cranePusherServo.write(craneStopSpeed);
    //initMovingAvg();
    while (!standbyClampMove && !doOp){
      changeUltrasonicMovingAvg();
      Serial.println(movingAvg);
    }  
    nonOpDelay(50);
    //linearServo.write(180);
    Serial.println("RIGHT");    
    cranePusherServo.write(0);
    while (standbyClampMove && !doOp){
          changeUltrasonicMovingAvg();
        Serial.println(movingAvg);

    }
    nonOpDelay(50);
    //linearServo.write(craneStopSpeed);
    cranePusherServo.write(craneStopSpeed);
    //initMovingAvg();
    Serial.println("OFF");    
    while (!standbyClampMove && !doOp){
    changeUltrasonicMovingAvg();
    Serial.println(movingAvg);

    } 
    nonOpDelay(50); 
    cranePusherServo.write(180);
        Serial.println("LEFT");    
    while (standbyClampMove && !doOp){
    changeUltrasonicMovingAvg();
    Serial.println(movingAvg);

    }       
    nonOpDelay(50); 
    cranePusherServo.write(craneStopSpeed);
    return;
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

void standbyMode3(){//ultrasonic testing

    rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
    rotatorOne.write(rotatorUpValueOne);
    
    cranePusherServo.write(craneStopSpeed);//keep crane still

    //cranePusherServo.write(craneStopSpeed);
    //initMovingAvg();

    while (digitalRead(snapActionPin) == LOW && !doOp){
    changeUltrasonicMovingAvg();
    Serial.println(inRangeInclusive(movingAvg,minPoleHeight,maxPoleHeight));
    }  
    nonOpDelay(50);
    //linearServo.write(180);
    
    cranePusherServo.write(0);
    while (digitalRead(snapActionPin) == HIGH && !doOp){
    changeUltrasonicMovingAvg();
    Serial.println(inRangeInclusive(movingAvg,minPoleHeight,maxPoleHeight));

    }
    nonOpDelay(50);
    //linearServo.write(craneStopSpeed);
    cranePusherServo.write(craneStopSpeed);
    //initMovingAvg();
    while (digitalRead(snapActionPin) == LOW && !doOp){
    changeUltrasonicMovingAvg();
    Serial.println(inRangeInclusive(movingAvg,minPoleHeight,maxPoleHeight));

    } 
    nonOpDelay(50); 
    cranePusherServo.write(180);
    while (digitalRead(snapActionPin) == HIGH && !doOp){
    changeUltrasonicMovingAvg();
    Serial.println(inRangeInclusive(movingAvg,minPoleHeight,maxPoleHeight));

    }       
    nonOpDelay(50); 
    cranePusherServo.write(craneStopSpeed);
    return;
    
}

void standbyMode4(){//ultrasonic testing

    rotatorTwo.write(rotatorUpValueTwo);//raise flapper non-blocking
    rotatorOne.write(rotatorUpValueOne);
    
    cranePusherServo.write(craneStopSpeed);//keep crane still

    //cranePusherServo.write(craneStopSpeed);
    //initMovingAvg();

    while (digitalRead(snapActionPin) == LOW && !doOp){
    changeUltrasonicMovingAvg();
    Serial.println(movingAvg);
    }  
    nonOpDelay(50);
    //linearServo.write(180);
    
    cranePusherServo.write(0);
    while (digitalRead(snapActionPin) == HIGH && !doOp){
    changeUltrasonicMovingAvg();
    Serial.println(movingAvg);

    }
    nonOpDelay(50);
    //linearServo.write(craneStopSpeed);
    cranePusherServo.write(craneStopSpeed);
    //initMovingAvg();
    while (digitalRead(snapActionPin) == LOW && !doOp){
    changeUltrasonicMovingAvg();
    Serial.println(movingAvg);

    } 
    nonOpDelay(50); 
    cranePusherServo.write(180);
    while (digitalRead(snapActionPin) == HIGH && !doOp){
    changeUltrasonicMovingAvg();
    Serial.println(movingAvg);

    }       
    nonOpDelay(50); 
    cranePusherServo.write(craneStopSpeed);
    return;
    
}

//this goes from the tire drop location to the location of tire pick-up (the starboard side of the correct ebump)
void resetClampPartOne(){

  //make sure that both flapper are raised
  rotatorTwo.write(rotatorUpValueTwo);
  rotatorOne.write(rotatorUpValueOne);
    

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

  //position the clamp immediately to the starboard side of the correct ebump pin for the next flappercase
  if (flapperCase > clampLoc){//move to the port if clamp is to the starboard of the correct side
    cranePusherServo.write(craneFastPort);
    Serial.println("PORT");
  } else {//move to the starboard if clamp is to the port of the correct side
    cranePusherServo.write(craneFastStarboard);
    Serial.println("STARBOARD");
  }

  //travel until the correct flapper ebump is hit
  if (flapperCase == 1){
    ///// DEBUG CODE
    //while (digitalRead(ebumpTwoPin) == LOW && doOp);
    //Serial.println("EBUMP ONE HIGH");
    //cranePusherServo.write(craneStopSpeed);    
    //opDelay(5000);
    /////
    while(digitalRead(ebumpOnePin) == LOW && doOp);
    
    
  } else if (flapperCase == 2){
    while(digitalRead(ebumpTwoPin) == LOW && doOp);
  } else if (flapperCase == 3){
    while(digitalRead(ebumpThreePin) == LOW && doOp);
  } else if (flapperCase == 4){
    while(digitalRead(ebumpFourPin) == LOW && doOp);
  }

  //if the clamp was moving starboardwards then keep moving to the starboard side of the ebump
  //note that movements over the electric bump are filtered for noise by sampling with a delay between
  
  if (flapperCase <= clampLoc){//moving starboardwards
    if (flapperCase == 1){
      while(digitalRead(ebumpOnePin) == HIGH && doOp){
        opDelay(opDelayAmount);
      }
      
    } else if (flapperCase == 2){
      while(digitalRead(ebumpTwoPin) == HIGH && doOp){
        opDelay(opDelayAmount);
      }
    } else if (flapperCase == 3){
      while(digitalRead(ebumpThreePin) == HIGH && doOp){
        opDelay(opDelayAmount);
      }
    } else if (flapperCase == 4){
      while(digitalRead(ebumpFourPin) == HIGH && doOp){
        opDelay(opDelayAmount);
      }
    }
    cranePusherServo.write(craneSlowPort);
    //now go port a bit to make sure it is on the edge of the bump
    if (flapperCase == 1){
      while(digitalRead(ebumpOnePin) == LOW && doOp);
    } else if (flapperCase == 2){
      while(digitalRead(ebumpTwoPin) == LOW && doOp);
    } else if (flapperCase == 3){
      while(digitalRead(ebumpThreePin) == LOW && doOp);
    } else if (flapperCase == 4){
      while(digitalRead(ebumpFourPin) == LOW && doOp);
    }
  }
  

  //stop the clamp at the location where it will pick up the tire from the flapper (aka to the starboard side of the correct ebump)
  cranePusherServo.write(craneStopSpeed);
  
  return;
}



const int servoLength = 2000; //used to be 14000

//pick up tire then this goes from the location of tire pickup to the outisde of the outermost ebump, where it will wait until a tire drop is needed
void resetClampPartTwo(){
  //init the servo motors
  rotatorTwo.write(rotatorUpValueTwo);//ensure flappers are raised flappers non-blocking
  rotatorOne.write(rotatorUpValueOne);
  cranePusherServo.write(craneStopSpeed);//keep crane still
  if (flapperCase == 3 || flapperCase == 4){//the following cases are for if the tire is to be picked up from flappers 3 or 4
    lowerFlapperTwo(); //lower the flapper corresponding to the current flapper case
    closeClamp(); //close the clamp 
    //move in the other direction away from the flapper all the way to the starboard-most ebump
    cranePusherServo.write(craneFastStarboard);
    Serial.println("ONE");
    while (digitalRead(ebumpOnePin) == LOW && doOp);
    Serial.println("TWO");
    while(digitalRead(ebumpOnePin) == HIGH && doOp){//traverse the ebump to the starboard side
      opDelay(opDelayAmount);
    }
    Serial.println("THREE");
    clampLoc = 0;// set the current location of the clamp
    cranePusherServo.write(craneStopSpeed);
    //rotatorTwo.write(rotatorUpValueTwo);
    raiseFlapperTwo();

    Serial.println("FOUR");

  } else {//if the current flapper is 1 or two 
    lowerFlapperOne(); //lower the flapper corresponding to the current flapper case
    //linearServo.write(180);//close the clamp
    //opDelay(servoLength);
    closeClamp(); //close the clamp

    cranePusherServo.write(craneFastPort);
    while (digitalRead(ebumpFourPin) == LOW && doOp);
    while(digitalRead(ebumpFourPin) == HIGH && doOp){//traverse the ebump to the port side
      opDelay(opDelayAmount);
    }
    clampLoc = 4;//set the current location of the clamp
    cranePusherServo.write(craneStopSpeed);
    //rotatorOne.write(rotatorUpValueOne);
    raiseFlapperOne();
  }  
  return;
}


//this keeps track of the location of the clamp as the clamp moves starboardwards
//clampLoc is a global variable that gets updated
//onBump is a bool that must be inputted through each loop iteration that this function is called from
bool updateClampLocStarboard(bool onBump){
      if (clampLoc == 4 && digitalRead(ebumpFourPin) == HIGH){
        onBump = true;
      } 
      if (clampLoc == 3 && digitalRead(ebumpThreePin) == HIGH){
        onBump = true;
      } 
      if (clampLoc == 2 && digitalRead(ebumpTwoPin) == HIGH){
        onBump = true;
      } 
      if (clampLoc == 1 && digitalRead(ebumpOnePin) == HIGH){
        onBump = true;
      }

      if (onBump && clampLoc == 4 && digitalRead(ebumpFourPin) == LOW){
        onBump = false;
        clampLoc = 3;
      }
      if (onBump && clampLoc == 3 && digitalRead(ebumpThreePin) == LOW){
        onBump = false;
        clampLoc = 2;
      }
      if (onBump && clampLoc == 2 && digitalRead(ebumpTwoPin) == LOW){
        onBump = false;
        clampLoc = 1;
      }
      if (onBump && clampLoc == 1 && digitalRead(ebumpOnePin) == LOW){
        onBump = false;
        clampLoc = 0;
      }
      return onBump;
}


//this goes from the outermost bump pin, scans for tire with ultrasonic sensor, and then drops tire at the right location
void dropTire(){ 
  if (flapperCase == 3 || flapperCase == 4){//the following cases are for if the tire is to be picked up from the starboard-most side

    //move the servo until the ultrasonic sensor sees the pole
    //the clamp should be starting from position 0, on the starboard side of ebump 1
    cranePusherServo.write(craneMediumPort);
    //opDelay(1000);//drive away from wall for the ultrasonic sensor
    initMovingAvg();
    //unsigned long startingTime = millis();
    while (!inRangeInclusive(movingAvg,minPoleHeight,maxPoleHeight) && doOp){//move until the ultrasonic sensor sees the pole
      /*if (millis()-startingTime > 2000){//drive away from wall for the ultrasonic sensor for 2 seconds
        changeUltrasonicMovingAvg();
      }*/
      changeUltrasonicMovingAvg();
      //Serial.println(movingAvg);
      Serial.println(movingAvg);

      ///////update the clamploc if the clamp hits the starboard side of one of the ebump pins
      if (digitalRead(ebumpOnePin) == HIGH){
        clampLoc = 1;
      } else if (digitalRead(ebumpTwoPin) == HIGH){
        clampLoc = 2;
      } else if (digitalRead(ebumpThreePin) == HIGH){
        clampLoc = 3;
      } else if (digitalRead(ebumpFourPin) == HIGH){
        clampLoc = 4;
      }
      /////ending the updating of clamploc
    }

    //now reverse a bit to align well with the pole
    cranePusherServo.write(craneSlowStarboard);
    bool passedTire = false;
    bool leaveLoop = false;
    bool onBump = false;
    while (!leaveLoop && doOp){//move until the ultrasonic sensor sees the pole
      changeUltrasonicMovingAvg();
      //Serial.println(clampLoc);
      Serial.println(movingAvg);
      
      ////update the clamploc if the clamp hits the starboard side of one of the ebump pins    
      onBump = updateClampLocStarboard(onBump);
      /////end of the updating of clamploc

      //check whether to end loop
      if (inRangeInclusive(movingAvg,minPoleHeight,maxPoleHeight)){
        passedTire = true;
      } 
      if (passedTire && inRangeInclusive(movingAvg, minGroundHeight, maxGroundHeight)){
        leaveLoop = true;
      }
    }


    
  } else {//if flapper is 1 or 2
    //the clamp should start in clamploc 4, port of the starboard side of ebump 4
    //move the crane starboardwards until the ultrasonic sensor sees the pole
    cranePusherServo.write(craneMediumStarboard);
    //delay(2000);//pass over the non-outermost ebump and away from wall
    bool onBump = false;
    initMovingAvg();
    unsigned long startingTime;//this variable is used for debounding after the ultrasonic sensor first sees the pole
    bool passedTire = false;
    bool leaveLoop = false;
    while (!leaveLoop && doOp){//move until the ultrasonic sensor sees the pole
      /*if ( ){//drive away from wall for the ultrasonic sensor for 0 seconds
        changeUltrasonicMovingAvg();
      }*/
      changeUltrasonicMovingAvg();
      //Serial.println(clampLoc);
      
          
      ////update the clamploc if the clamp hits the starboard side of one of the ebump pins    
      onBump = updateClampLocStarboard(onBump);
      /////end of the updating of clamploc

      //check whether to end loop
      if (!passedTire && inRangeInclusive(movingAvg,minPoleHeight,maxPoleHeight)){
        passedTire = true;
        startingTime = millis();
      } 
      if (passedTire && inRangeInclusive(movingAvg,minGroundHeight,maxGroundHeight)){
        Serial.println(millis() - startingTime);
      } 
      //Serial.println(movingAvg);
      if (passedTire){
        Serial.println("PASSED");
      }
      if (passedTire && inRangeInclusive(movingAvg, minGroundHeight, maxGroundHeight) && millis() - startingTime > 150){
        leaveLoop = true;
        Serial.println("TIME:");
        Serial.println(millis()-startingTime);
      }
    }
    //cranePusherServo.write(craneStopSpeed);
    //opDelay(5000);

    //now reverse a bit to go to the right position
    cranePusherServo.write(craneMediumPort);
    //opDelay(1000);//drive away from wall for the ultrasonic sensor
    startingTime = millis();
    while (!inRangeInclusive(movingAvg,minPoleHeight,maxPoleHeight) && doOp){//move until the ultrasonic sensor sees the pole
      //if (millis()-startingTime > 2000){//drive away from wall for the ultrasonic sensor for 2 seconds
      //  changeUltrasonicMovingAvg();
      //}
      changeUltrasonicMovingAvg();
      //Serial.println(movingAvg);
      Serial.println(movingAvg);

      ///////update the clamploc if the clamp hits the starboard side of one of the ebump pins
      if (digitalRead(ebumpOnePin) == HIGH){
        clampLoc = 1;
      } else if (digitalRead(ebumpTwoPin) == HIGH){
        clampLoc = 2;
      } else if (digitalRead(ebumpThreePin) == HIGH){
        clampLoc = 3;
      } else if (digitalRead(ebumpFourPin) == HIGH){
        clampLoc = 4;
      }
      /////ending the updating of clamploc
    }    
    //now re-reverse a bit to align well with the pole
    cranePusherServo.write(craneSlowStarboard);
    passedTire = false;
    leaveLoop = false;
    onBump = false;
    while (!leaveLoop && doOp){//move until the ultrasonic sensor sees the pole
      changeUltrasonicMovingAvg();
      //Serial.println(clampLoc);
      Serial.println(movingAvg);
      
      ////update the clamploc if the clamp hits the starboard side of one of the ebump pins    
      onBump = updateClampLocStarboard(onBump);
      /////end of the updating of clamploc

      //check whether to end loop
      if (inRangeInclusive(movingAvg,minPoleHeight,maxPoleHeight)){
        passedTire = true;
      } 
      if (passedTire && inRangeInclusive(movingAvg, minGroundHeight, maxGroundHeight)){
        leaveLoop = true;
      }
    }
    //cranePusherServo.write(180);
    //go portwards a bit to be at the right position
    /*while (!inRangeInclusive(movingAvg,minPoleHeight,maxPoleHeight) && doOp){//move until the ultrasonic sensor sees the pole
      //Serial.println(movingAvg);
      //Serial.println(clampLoc);

      ///////update the clamploc if the clamp hits the starboard side of one of the ebump pins
      if (digitalRead(ebumpOnePin) == HIGH){
        clampLoc = 1;
      } else if (digitalRead(ebumpTwoPin) == HIGH){
        clampLoc = 2;
      } else if (digitalRead(ebumpThreePin) == HIGH){
        clampLoc = 3;
      } else if (digitalRead(ebumpFourPin) == HIGH){
        clampLoc = 4;
      }
      /////ending the updating of clamploc
    }*/
    
  }
  
  //the pole has been found so now drop the tire
  Serial.println("pole found");
  cranePusherServo.write(craneStopSpeed);//stop the clamp now that the ultrasonic sensor has found the pole
  openClamp();//drop the tire
  
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
    Serial.println((char)x); // Print to serial output as char (ASCII representation)
    if (x == '1'){
       tireDropRequested = true;
       finishedYet = false;
    } else if (x == '2'){//this is to signify the start of an operation
      firstTimeThrough = true;//this signifies that the operation has just begun
      doOp = true;
    } else if (x == '3'){ // this is to signify the end of an operation... aka the operation is aborted or time is up
      doOp = false; 
      standbyClampMove = false;//make sure the clamp stays still after the operation
    } else if (x == 'A'){
      standbyClampMove = !standbyClampMove;
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
