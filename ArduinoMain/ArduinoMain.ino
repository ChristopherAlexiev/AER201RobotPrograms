/* i2c components based off of sample code for i2c from Michael Ding and Tyler Gamvrelis
 */

#include <Wire.h>

void setup(){
    Wire.begin(8); // Join I2C bus with address 8
  
    // Register callback functions
    Wire.onReceive(receiveEvent); // Called when this slave device receives a data transmission from master
    Wire.onRequest(requestEvent); // Called when master requests data from this slave device
  
    // Open serial port to PC (hardware UART)
    Serial.begin(9600);      
}

volatile bool send_to_pic = false;
volatile uint8_t incomingByte;
volatile bool doTireDrop;
volatile int snapSwitchTriggered = 0;
volatile int finishedYet = false;
volatile int tireDropRequested = false;

void loop(){
  
/*
  enum tireStacks {one, two, three, four};
  enum tireStacks currentTireStack = one;
  enum arduinoStates {preparingTire, bipolarPush, craneMoveToTire, craneMoveToPole, flapDown, , complete};
  enum operationStates currentArduinoState = idle;
  
    while (1){
    
    //all stacks up, crane open
    
    //START
    
    //bring crane to current tire stack with snap action sensor
    while(!snapSwitchTriggered){
        positionCraneMoveServo ++;
        //add distance to stepper motor that controls crane;
    }
    
    //lower flapper of current tire stack
    while (position is greater than 0 degrees){
      //lower flapper stepper motor to a lower position   
      switch (currentTireStack){//FINISH DETAILS!!!!!
            case one: //moving the robot to the next pole    
                positionFlapperLeft --;
        break;
            case two:
        positionFlapperLeft --;
        break;
      case three:
        positionFlapperRight --;
        break;
      case four:
        positionFlapperRight --;
        break;
    }
    
    //close crane clamp
    for(enough degrees to close crane){
        positionClampServo ++;
        //add distance to stepper motor that controls crane;
    }
    
    //move crane away from lowered side to the end of robot
    while(1){
        
      //lower flapper stepper motor to a lower position   
      switch (currentTireStack){//FINISH DETAILS!!!!!
            case one: //moving the robot to the next pole    
                positionCraneMoveServo ++;
        break;
            case two:
        positionCraneMoveServo ++;
        break;
      case three:
        positionCraneMoveServo --;
        break;
      case four:
        positionCraneMoveServo --;
        break;
      if(snapSwitchTriggered){
        bumpsDetected ++;
      }
      if (currentTireStack and current number of bumps detected are right){
        break;
      }
    }
    
    //raise flapper of current tire stack
    while (position is less than 0- degrees){
      //raise flapper stepper motor to a lower position   
      switch (currentTireStack){//FINISH DETAILS!!!!!
            case one: //moving the robot to the next pole    
                positionFlapperLeft ++;
        break;
            case two:
        positionFlapperLeft ++;
        break;
      case three:
        positionFlapperRight ++;
        break;
      case four:
        positionFlapperRight ++;
        break;
    }
    */
    //wait for tireDrop to equal true... ie for the PIC to send a tire drop request
    while (!tireDropRequested){
    }
    tireDropRequested = false;

    delay(1000);//simulate tire drop with a delay for now
    
    finishedYet = true;
    /*
    //scan stacks left to right until ultrasonic sensor sees something (for now)
    //move crane from the far side to the pole
    while(ultrasonic reading is big){
        
      //lower flapper stepper motor to a lower position   
      switch (currentTireStack){//FINISH DETAILS!!!!!
            case one: //moving the robot to the next pole    
                positionCraneMoveServo ++;
        break;
            case two:
        positionCraneMoveServo ++;
        break;
      case three:
        positionCraneMoveServo --;
        break;
      case four:
        positionCraneMoveServo --;
        break;
      
    }
    
    //open crane clamp
    for(enough degrees to open crane){
        positionClampServo --;
        //add distance to stepper motor that controls crane clamp;
    }
    
    finishedYet = true;
    
    currentTireStack ++;
    //if currentT == 1 then push all with the bipolar motor
    if (currentTireStack == 1){
      set bipolar motor direction and power variables;
      for (number of steps required){
        servo.step();
      }
    }
    
    */
    /*
    // If we should send to the PIC, then we wait to receive a byte from the PC
    if (send_to_pic && Serial.available() > 0 && !incomingByte) {
        incomingByte = Serial.read();
    }
    */
}

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
      Serial.print("TRUE");
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
