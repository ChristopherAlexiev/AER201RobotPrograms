/**
 * keypad and LCD code credit to  Michael Ding and Tyler Gamvrelis
 * 

 * Preconditions:
 * @pre Character LCD in a PIC socket
 * @pre Co-processor is running default keypad encoder program
 */

#include "xc.h"
#include <math.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "configBits.h"
#include "lcd.h"
#include "I2C.h"

//global variables
int logs[5][48];//five logs(four saved and one current), with 48 info pieces per log (25*3 pole infos + 3 intro infos)
    //logs format:
    //operation time, tires supplied, number of poles, (tires deployed on pole, tires on pole after op, distance to pole) * the pole number
    

const char keys[] = "123A456B789C*0#D";
//const char msg1[] = "Standby";
//const char msg2[] = "Operation";

//Global variable used in interrupts
volatile bool keyPressed = false;
volatile int motorADistance = 0;
volatile int motorBDistance = 0;
volatile unsigned long millisecondsMeasured;
volatile bool topBreakBeamTriggeredChange = 0;
volatile bool bottomBreakBeamTriggeredChange = 0;
volatile bool motorBDirection = true;//the rotation direction of motor B
volatile bool motorADirection = true;
volatile bool encoderBInterruptLast;
volatile bool encoderAInterruptLast;



void doDisplayLog(int logNumber){
    lcd_home();
    /*
    printf("%s", "Displaying log");
    __delay_ms(4000);
    *///REMOVEDDD
    int currentPole = 1;
    enum displayStates {opTime, pole};//one is the most recent log
    enum displayStates currentDisplayState = opTime; 
    while (1){
        switch (currentDisplayState){
            case opTime:
                //display standby screen
                lcd_home();
                printf("%s%d%s", "OP. TIME ",logs[logNumber][0], "s");//ADD VARIABLE LATER
                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                printf("%s%d", "TIR. SUPPLIED ",logs[logNumber][1]);
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("%s","(1) POLE INFO");
                lcd_set_ddram_addr(LCD_LINE4_ADDR);
                printf("%s","(2) SAVE & EXIT");
                break;
            case pole:
                lcd_home();
                printf("%s%d%s%d", "POLE ", currentPole,"/", logs[logNumber][2]);//FIX LATER
                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                printf("%s%d","TIR. DEPLOY: ", logs[logNumber][3+(currentPole-1)*2]);
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("%s%d","TIRE. ON POLE: ", logs[logNumber][4+(currentPole-1)*2]);
                lcd_set_ddram_addr(LCD_LINE4_ADDR);
                printf("%s","1<- 2-> 3MENU");//MANUALLY TYPE ARROWS WITH BITS LATER
                break;
        }
        //UPDATE SCREEN
        if(keyPressed){
            
            
            keyPressed = false; // Clear the flag
            
            // Write key press data to bottom line of LCD
            unsigned char keypress = (PORTB & 0xF0) >> 4;
            char keyValue = keys[keypress];
            lcd_clear(); //clear lcd for next state displayed to be correct
            switch (currentDisplayState){
                case opTime:
                    if (keyValue == '1'){
                        currentDisplayState = pole;
                    } else if (keyValue == '2'){
                        return;
                    } 
                    break;
                case pole:
                    if (keyValue == '3'){
                        currentDisplayState = opTime;
                        currentPole = 1;
                        //should also have if statement for adding and subtracting from currentPole later
                    } else if (keyValue == '2'){
                        if(currentPole < logs[logNumber][2]){
                            currentPole++;
                        }
                    } else if (keyValue == '1') {
                        if (currentPole > 1){
                            currentPole--;
                        }
                    }
                    break;
            }
            /*lcd_set_ddram_addr(LCD_LINE2_ADDR);
            putch(keys[keypress]);*/
        }
    __delay_ms(1);    
    }
    return;
}

//DELAY FUNCTION FOR OPERATIONS
//a function that delays for some number of milliseconds that will exit in case the time of the operation is up
void opDelay( int millis, unsigned long maxOperationTime){
    unsigned long startTime = millisecondsMeasured;
    while (millisecondsMeasured < maxOperationTime && (millisecondsMeasured - startTime) < millis){ 
    }
    return;
}

//encoder distance measurement stuff (code ideas from https://www.dfrobot.com/wiki/index.php/Micro_DC_Motor_with_Encoder-SJ02_SKU:_FIT0458)
void updateMotorBEncoderOLD(){
  int interruptPinState = PORTBbits.RB2;//the interrupt pin for motor B
  if(encoderBInterruptLast == 0 && interruptPinState==1){
    int val = PORTEbits.RE2;//the direction input from the encoder
    if(val == 0 && motorBDirection){
      motorBDirection = false; //Reverse
    }
    else if(val == 1 && !motorBDirection){
      motorBDirection = true;  //Forward
    }
  }
  encoderBInterruptLast = interruptPinState;
 
  //change the encoder distance measured for motor B
  if(!motorBDirection){
      motorBDistance++;
  }else{
      motorBDistance--;
  }
}

void updateMotorBEncoder(){
  int interruptPinState = PORTBbits.RB2;//the interrupt pin for motor B
  int directionPin = PORTCbits.RC0;//the direction input from the encoder
  if(directionPin == interruptPinState){
    motorBDistance++;
  } else {
    motorBDistance--;
  }
}

void updateMotorAEncoderOLD(){
  int interruptPinState = PORTBbits.RB0;//the interrupt pin for motor B
  if(encoderAInterruptLast == 0 && interruptPinState==1){
    int val = PORTEbits.RE1;//the direction input from the encoder
    if(val == 0 && motorADirection){
      motorADirection = false; //Reverse
    }
    else if(val == 1 && !motorADirection){
      motorADirection = true;  //Forward
    }
  }
  encoderAInterruptLast = interruptPinState;
 
  //change the encoder distance measured for motor B
  if(!motorADirection){
      motorADistance++;
  }else{
      motorADistance--;
  }
}

void updateMotorAEncoder(){
  int interruptPinState = PORTBbits.RB0;//the interrupt pin for motor B
  int directionPin = PORTEbits.RE1;//the direction input from the encoder
  if(directionPin == interruptPinState){
    motorADistance--;
  } else {
    motorADistance++;
  }
}

void EncoderInit(){
  motorBDirection = true;//default -> Forward  
  motorADirection = true;
  
  //set up motor encoder pin direction input
  TRISCbits.TRISC0 = 1;//motor B direction
  TRISEbits.TRISE1 = 1; //motorA direction
  TRISBbits.TRISB2 = 1;//motor B distance
  TRISBbits.TRISB0 = 1;//motor A distance
  
  //set up encoder interrupt pin for RB2
  INT2IE = 1;
  //and RB0
  INT0IE = 1;
}

//CLOCK STUFF

#define OSC_CLOCKS_PER_INSTRUCTION_CYCLE (4ul)
#define TIMER0_PRESCALE_COUNT (2ul)
#define TIMER0_COUNTS_PER_HALF_SECOND (65536ul - (((_XTAL_FREQ / OSC_CLOCKS_PER_INSTRUCTION_CYCLE)/TIMER0_PRESCALE_COUNT)/1000ul))

/*this function not needed for external oscillator... aka for this program*/
void CLK_INIT( void ){
    OSCCONbits.IRCF = 0b100; //1 MHz
}

//this function from https://www.microchip.com/forums/m1075989.aspx
void TIMER_INIT( void ){
    T0CONbits.TMR0ON  = 0;      //stop TIMER0 counter
    T0CONbits.T08BIT  = 0;      //select 16-bit mode
    T0CONbits.T0CS    = 0;      //select the internal clock to drive timer0//this is actually clk0 which is one fourth of external clock signal
    T0CONbits.PSA     = 0;      //use prescaler for TIMER0
    T0CONbits.T0PS    = 0b000;  //assign the 1:64 pre-scaler to TIMER0 
    TMR0H = TIMER0_COUNTS_PER_HALF_SECOND >> 8;
    TMR0L = TIMER0_COUNTS_PER_HALF_SECOND;
    T0CONbits.TMR0ON  = 1;      //enable TIMER0
    INTCONbits.TMR0IE = 1;      //Enable the TIMER0 overflow interrupt
    //INTERUPT ENABLE
    INTCONbits.GIE    = 1;      //GLOBAL INTERUPT ENABLE
    INTCONbits.PEIE   = 1;      //PERIPHERAL INTERUPT ENABLE; TIMER0 = PERIPHERAL
}


//set timer interrupt for 1 second
void initSecondTimer(){
    //set millisecondsMeasured to zero
    millisecondsMeasured = 0ul;
    //set timer interrupt:
    T08BIT = 0; //set to 16 bit timer
    T0PS2 = 1;//set timer to 256 prescaler
    T0PS1 = 1;
    T0PS0 = 1;
    PSA=0;//set timer to use prescaler
    //ticks required in a second are 1000000/256=3906 with prescaler 256
    //ticks left after subtracting from 26 bit timer are 65536-3906=61630
    //therefore 61630 is the timer preload value
    TMR0H = 0xF0;
    TMR0L = 0xBE;               // Timer0 preload value
    TMR0IE = 1;// enable timer0 interrupt 
    TMR0ON = 1; //turn on timer 0
         
    return;
}


void stopTimer(){//stop the 16 bit timer from counting seconds
    TMR0IE = 0;//disable timer0 interrupt
}
/*
do an operation 
 
*/

//initialize the motor pwm
void init_motor_PWM(){
        // Set internal oscillator to run at 8 MHZ
    //OSCCON = OSCCON | 0b01110000; 
    
    // Enable PLL for the internal oscillator, the processor now runs at 32 MHz
    //OSCTUNEbits.PLLEN = 1; 
    
    // Disable output from PWM pin while we are setting up PWM
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC1 = 1;
     
    // Configure PWM frequency, 3.1 kHz. See datasheet pg. 149, equation 16-1
    const unsigned long FREQUENCY = 1000ul;
    const unsigned char TIMER2_PRESCALER = 16;
    PR2 = (_XTAL_FREQ / (FREQUENCY * 4 * TIMER2_PRESCALER)) - 1;
    
    // Configure CCP1CON, single output mode, all active high
    P1M1 = 0;
    P1M0 = 0;
    CCP1M3 = 1;
    CCP1M2 = 1;
    CCP1M1 = 0;
    CCP1M0 = 0;
    //configure CCP2CON
    CCP2M3 = 1;
    CCP2M2 = 1;
    CCP2M1 = 0;
    CCP2M0 = 0;

    // Set timer 2 prescaler to 16
    T2CKPS0 = 1;
    T2CKPS1 = 1;

    // Enable timer 2
    TMR2ON = 1;

    // Enable PWM output pin since setup is done
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC1 = 0;
    
    return;
}

void set_pwm_duty_cycle(float dutyA, float dutyB){
    if((dutyA >= 0) && (dutyA <= 100.0) && (dutyB >= 0) && (dutyB <= 100.0)){
        // Our pulse width cannot exceed the period of the wave. First we
        // compute this upper limit for the duty cycle registers (max_duty_val),
        // then we compute a percentage of this (duty_val) as per the argument
        // passed in. See datasheet pg 150-151 (equation 16-2, and figure 16-2)
        unsigned short max_duty_val = PR2 + 1;//technically this should be times 4, but this is fixed below
        //BELOW SAMPLE CODE IS NOT WORKING
        // unsigned short duty_val = (unsigned short)(
        //    (duty / 100.0) * (float)max_duty_val
        //);
        //THIS CODE WORKS
        unsigned short duty_valA = (unsigned short)((dutyA *4 / 100.0) * (float)max_duty_val);
        unsigned short duty_valB = (unsigned short)((dutyB *4/ 100.0) * (float)max_duty_val);
   
        
        // pwm to port rc2
        CCP1X = duty_valA & 2; // Set the 2 least significant bit in CCP1CON register
        CCP1Y = duty_valA & 1;
        CCPR1L = duty_valA >> 2; // Set rest of the duty cycle bits in CCPR1L

        
        //pwm to port rc1
        CCP2X = duty_valB & 2; // Set the 2 least significant bit in CCP2CON register
        CCP2Y = duty_valB & 1;
        CCPR2L = duty_valB >> 2; // Set rest of the duty cycle bits in CCPR2L

    }
}

void setMotorSpeeds(int motorASpeed, bool Aforward, bool Bforward, int motorBSpeed){
    if (Aforward){
        LATAbits.LATA0 = 1;
        LATAbits.LATA1 = 0;
    } else {
        LATAbits.LATA0 = 0;
        LATAbits.LATA1 = 1;
    }
    if (Bforward){
        LATAbits.LATA2 = 1;
        LATAbits.LATA3 = 0;
    } else {
        LATAbits.LATA2 = 0;
        LATAbits.LATA3 = 1;
    }
    //LATCbits.LATC7 = 1;
    set_pwm_duty_cycle((float) motorASpeed, (float) motorBSpeed);
    return;
}


//this function positions the robot on the pole and  returns the number of tires on the pole
int tirePositioning(unsigned long maxOperationTime){
    lcd_clear();
    lcd_home();
    printf("TIRE POSITIONING");
    int topBreakbeam;
    int bottomBreakbeam;
    int distanceRecordedTop;
    int startDistanceTop;
    int distanceRecordedBottom;
    int startDistanceBottom;
    int topPreviousState = 1; //assume the break beam was high before the function started.
    int bottomPreviousState = 1;//assume the break beam was high before the function started
    
    /////analyze the pole and collect data on the distances travelled while the breakbeam lasers are triggered
    while(millisecondsMeasured < maxOperationTime){
        topBreakbeam = PORTBbits.RB4;
        bottomBreakbeam = PORTBbits.RB5;
        if (topBreakbeam == 0 && topPreviousState == 1){
            startDistanceTop = motorADistance;
        }
        if (bottomBreakbeam == 0 && bottomPreviousState == 1){
            startDistanceBottom = motorADistance;
        }
        if(!topBreakbeam){//if still seeing the top tire then continue to increase the top tire distance
            distanceRecordedTop = abs(motorADistance - startDistanceTop);
        }
        if(!bottomBreakbeam){//if still seeing the bottom tire then continue to increase the bottom tire distance
            distanceRecordedBottom = abs(motorADistance - startDistanceBottom);            
        }
        if(topBreakbeam && bottomBreakbeam){//leave the loop if the robot has passed by the pole and its tires
            break;
        }
        topPreviousState = topBreakbeam;
        bottomPreviousState = bottomBreakbeam;
    }
    /////position the robot so that the tire dropping mechanism is on top of the poles
    if (distanceRecordedTop < 170){// if there is no top tire then go until the pole is hit by the top break beam
        setMotorSpeeds(0, true, true, 0);
        opDelay(100, maxOperationTime);
        while(millisecondsMeasured < maxOperationTime && PORTBbits.RB4 == 1){
            setMotorSpeeds(30, false, false, 30);
        }
        setMotorSpeeds(0, true, true, 0);
    } else {//if there is a top tire then go backwards until the bottom tire is detected and then halfway through it
        setMotorSpeeds(0, true, true, 0);        
        opDelay(100, maxOperationTime);
        while(millisecondsMeasured < maxOperationTime && PORTBbits.RB4 == 1){
            setMotorSpeeds(30, false, false, 30);
        }
        int startDistance = motorADistance;
        while(millisecondsMeasured < maxOperationTime && abs(startDistance - motorADistance) < distanceRecordedBottom/2){
            setMotorSpeeds(30, false, false, 30);
        }
        setMotorSpeeds(0, true, true, 0);
    }
    
    /////return the correct number of tires that were on the pole
    if (distanceRecordedTop  < 170 && distanceRecordedBottom < 170){ // if there are no tires on the pole, typically the pole width is about 130
        return 0;
    } else if (abs(distanceRecordedTop - distanceRecordedBottom) > 180){ //the breakbeam separation for if there is one tire is about 300 encoder units
        return 1; //one tire is on the pole if there is a big difference between the two break beam distances
    } else {
        return 2;//two tires are on the pole if there is a small difference between the two break beam distances
    }
}
/*
void addPoleToLog(int tiresToDrop){
    return;
}
*/
void sendArduinoTireDropRequest(){
    //unsigned char mem[3]; // Initialize array to check for triple-A sequence
    //unsigned char counter = 0; // Increments each time a byte is sent
    //unsigned char data; // Holds the data to be sent/received
    
    unsigned char data = '1'; //char 1 is to represent a tire drop request
    
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
    I2C_Master_Write(data); // Write key press data
    I2C_Master_Stop();
/*
    while(1) {
        if(send){

            
            while (keyPressed == false){
                continue;
            }
            keyPressed = false;
            unsigned char keypress = (PORTB & 0xF0) >> 4;

            
            data = keys[keypress];
            
            I2C_Master_Start(); // Start condition
            I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
            I2C_Master_Write(data); // Write key press data
            I2C_Master_Stop();

            // Check for a triple-A sequence. If this sequence occurs, switch 
            // the PIC to receiver mode. To switch back to transmitter mode,
            // reset the PIC
            mem[counter] = data;
            counter++;
            counter = (counter == 3) ? 0 : counter;
            if((mem[0] == 'A') && (mem[1] == 'A') && (mem[2] == 'A')){
                send = false;
            }
            //leave doOperation())
            if((mem[0] == 'D') && (mem[1] == 'D') && (mem[2] == 'D')){
                break;
            }
        }
        else{
            // Receive data from Arduino and display it on the LCD
            I2C_Master_Start();
            I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
            data = I2C_Master_Read(NACK); // Read one char only
            I2C_Master_Stop();
            if(data){
                putch(data);
            }
        }
    }
 */
    return;
}

void sendArduinoTireOperationStartMessage(){
    //unsigned char mem[3]; // Initialize array to check for triple-A sequence
    //unsigned char counter = 0; // Increments each time a byte is sent
    //unsigned char data; // Holds the data to be sent/received
    
    unsigned char data = '2'; //char 1 is to represent a tire drop request
    
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
    I2C_Master_Write(data); // Write key press data
    I2C_Master_Stop();
    return;
}

void sendArduinoAbortOperationMessage(){
    //unsigned char mem[3]; // Initialize array to check for triple-A sequence
    //unsigned char counter = 0; // Increments each time a byte is sent
    //unsigned char data; // Holds the data to be sent/received
    
    unsigned char data = '3'; //char 1 is to represent a tire drop request
    
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
    I2C_Master_Write(data); // Write key press data
    I2C_Master_Stop();
    return;
}


bool requestIsTireDropDone(){//send request to arduino to see if the tire drop is complete
    I2C_Master_Start();
    I2C_Master_Write(0b00010001); // 7-bit Arduino slave address + Read
    unsigned char data = I2C_Master_Read(NACK); // Read one char only
    I2C_Master_Stop();
    if(data == '1'){
        return true;
    }
    return false;
}
/*
void adjustCourse(){
    return;
}

void returnHome(){
    return;
}

checkForPolesInWay(int* leftPole, int* rightPole){
    return;
}
*/

//non-blocking error corrected PID speed setting
//turn ratio is greater than 1 for motor B going fast
//derivative data is an array that holds the last error [0] and the integral [1]
void PIDCorrectedMove(int goalSpeed, float turnRatio, int motorAStartDistance, int motorBStartDistance, float kp, float kd, float ki, int * derivativeData){
    int error = (int)((motorADistance - motorAStartDistance)-(motorBDistance - motorBStartDistance)/turnRatio);
    int derivative = error - derivativeData[0];
    derivativeData[0] = error;
    int integral = derivativeData[1] + error;
    int correctionTerm = (int)(error*kp + derivative*kd + integral*ki);
    int ASpeed = goalSpeed - correctionTerm;
    int BSpeed = (int)((goalSpeed*turnRatio + correctionTerm));
    
    if (ASpeed > 100){
        ASpeed = 100;
    } 
    if (ASpeed < 0){
        ASpeed = 0;
    }
    if (BSpeed > 100){
        BSpeed = 100;
    }
    if (BSpeed < 0){
        BSpeed = 0;
    }
    
    setMotorSpeeds(ASpeed , true, true, BSpeed);
    return;
}



//non-blocking error corrected speed setting
//turn ratio is greater than 1 for motor B going fast
void errorCorrectedMove(int goalSpeed, float turnRatio, int motorAStartDistance, int motorBStartDistance, float correctionConstant){
    int error = (int)((motorADistance - motorAStartDistance)-(motorBDistance - motorBStartDistance)/turnRatio);
    int ASpeed = (int)(goalSpeed - error*correctionConstant);
    int BSpeed = (int)((goalSpeed*turnRatio + error*correctionConstant));
    
    if (ASpeed > 100){
        ASpeed = 100;
    } 
    if (ASpeed < 0){
        ASpeed = 0;
    }
    if (BSpeed > 100){
        BSpeed = 100;
    }
    if (BSpeed < 0){
        BSpeed = 0;
    }
    
    setMotorSpeeds(ASpeed , true, true, BSpeed);
    return;
}

//function to drive forward for some number of degrees 
//meant to be used in doOperation
//breaks if operation time limit is reached
//distance is in degrees
//stops the motors after
void opErrorCorrectionDegrees(int goalSpeed, int turnRatio, float correctionConstant, int distanceDegrees, unsigned long maxOperationTime){
    int motorAStartDistance = motorADistance;
    int motorBStartDistance = motorBDistance;

    //change the startDistance after every movement in dooperation
    
    while (millisecondsMeasured < maxOperationTime && abs(motorADistance - motorAStartDistance) < distanceDegrees){ 
        errorCorrectedMove(goalSpeed, turnRatio, motorAStartDistance, motorBStartDistance, correctionConstant);
    }
    return;
}


void doOperation(){
    // Write the address of the slave device, that is, the Arduino Nano. Note
    // that the Arduino Nano must be configured to be running as a slave with
    // the same address given here. Note that other addresses can be used if
    // desired, as long as the change is reflected on both the PIC and Arduino
    // ends 
    I2C_Master_Start();
    I2C_Master_Write(0b00010000); // 7-bit Arduino slave address + write
    I2C_Master_Stop();

    sendArduinoTireOperationStartMessage();//send a character to the arduino to signify the start of the operation
    //set timer for 3 minutes
    //initSecondTimer();//start the timer so millisecondsMeasured start to get updated
    //  CLK_INIT();
    TIMER_INIT();
    millisecondsMeasured = 30000ul;
    //set variables
	int goalSpeed = 20;
	int motorASpeed = 30;
	int motorBSpeed = 30;
	int errorScaleFactor = 1;
	int error = 0;
	int currentAngle = 0;
	enum operationStates {moveForward, poleFinding, tireDrop, courseAdjustment, returnHome, complete, leavePole};
	enum operationStates currentOperationState = moveForward;
	int leftRangeFinder = 0;
	int rightRangeFinder = 0;
	int tiresToDrop = 0;
    int minimumSafeDistanceToPole;
    int fourMetreEquivalent;
    unsigned long timeInOperation = 180000ul; //length in milliseconds
    int motorAStartDistance = motorADistance;//RESET TO 0 AFTER EVERY MOVEMENT
    int motorBStartDistance = motorBDistance;//RESET TO 0 AFTER EVERY MOVEMENT

    int PIDData[2] = {0,0};//this is the last error and the integral in one data structure, RESET TO {0,0} after every movement;
    bool topLaserState;
    bool topLaserStatePrev;
    bool bottomLaserState;
    bool bottomLaserStatePrev;
    bool isDoneDrop;
    init_motor_PWM();
    int counted = 0;
    int badCount = 0;
    int badCountBot = 0;
    
    //poleData
    int poleNumber = 0;
    int tiresDeployedOnPole = 0;
    int tiresOnPoleAfterOp = 0;
    int tiresOnPole = 0;//initial number of tires on pole
    
    int totalTiresSupplied = 0;

    int distanceToPole = 0;
    int distanceFromLastPole = 0;

    
    //NOTE...
    //MAKE SURE TO RESET PIDData, motorAStartDistancte, and motorBStartDistance before movements
    
    //initialize pins for operation
    TRISAbits.TRISA0 = 0;//pin for motor A direction
    TRISAbits.TRISA1 = 0;//pin for motor A direction
    TRISAbits.TRISA2 = 0;//pin for motor B direction
    TRISAbits.TRISA3 = 0;//pin for motor B direction
    
    //TRISCbits.TRISC7 = 0;//pin for motor direction
    TRISAbits.TRISA4 = 0;//pin for disabling keypad
    LATAbits.LATA4 = 1;///disable the keypad during the operation by putting voltage to the kpd bit
    unsigned long timeStart = millisecondsMeasured;
    while(millisecondsMeasured < timeStart + 100){//wait to let the B bits settle after turning off the keypad before using break beams
     //let break beams settle   
    }

    topLaserState = PORTBbits.RB4;//set top laser state variable to the current state of the laser
    topLaserStatePrev = topLaserState;
    bottomLaserState = PORTBbits.RB5;//set top laser state variable to the current state of the laser
    bottomLaserStatePrev = bottomLaserState;
    
    //RBIE = 1; //enable the on change interrupts for the laser break beams
    
    //FIX FIND OUT HOW TO	enable the on change interrupts in PORTRB4 and PORTRB5;
    //FIX finish working on timer interrupt
        // Main loop

    bool send = true;

    //LATCbits.LATC7 = 1;
    /*LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 1;*/
    while (1){
        counted ++;
        topLaserStatePrev = topLaserState;
        topLaserState = PORTBbits.RB4;
        bottomLaserStatePrev = bottomLaserState;
        bottomLaserState = PORTBbits.RB5;

        //__delay_ms(1);
        switch (currentOperationState){
            case moveForward: //moving the robot to the next pole
                if (counted%100 == 0){
                lcd_clear();//prepare screen for next state
                lcd_home();
                printf("%lu", millisecondsMeasured);
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("%d", topLaserState);
                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                printf("B: %d", motorBDistance);
                lcd_set_ddram_addr(LCD_LINE4_ADDR);
                printf("A: %d", motorADistance);
                }
                //printf("hello");
                /*error = motorADistance - motorBDistance;//proportional distance error correction
				motorASpeed = goalSpeed - error*errorScaleFactor;
				motorBSpeed = goalSpeed + error*errorScaleFactor;*/
				/*testing nonpwm, also comment out init_motor_pwm
                TRISCbits.TRISC2 = 0;
                TRISCbits.TRISC1 = 0;
                
                LATCbits.LATC1 = 1;
                LATCbits.LATC2 = 1;
                
                LATAbits.LATA2 = 0;
                LATAbits.LATA3 = 1;

                LATAbits.LATA2 = 1;
                LATAbits.LATA3 = 0;*/
                //PIDCorrectedMove(goalSpeed, 1, motorAStartDistance, motorBStartDistance, 0.1, 0, 0, PIDData);
                errorCorrectedMove(goalSpeed, 1, motorAStartDistance, motorBStartDistance, 0.1);
                //setMotorSpeeds(30, true, true, 30); //write new speeds to PORTC1 and PORTC2 PWM pins; 
                break;
            case poleFinding: //finding out the number of tires on the pole and positioning the robot by the pole
                //set up pole data:
                distanceFromLastPole = motorADistance - distanceToPole;
                distanceToPole = motorADistance;

                poleNumber++;
                
                //find out the number of tires on the pole and position the robot to the pole
                tiresOnPole = tirePositioning(timeInOperation);
				lcd_clear();//prepare screen for next state
                lcd_home();
                printf("TIRE DROP");
                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                printf("%d on pole", tiresOnPole);
                ////TEST CODE
                setMotorSpeeds(0, true, true, 0);
                /*while(millisecondsMeasured < timeInOperation){
                    
                }*/
                ////END OF TEST CODE
                

                
                
				sendArduinoTireDropRequest();
                //tiresToDrop = 1 or 2 depending on he number needed on the pole;

                tiresToDrop = 1;//hard-coded for now
                currentOperationState = tireDrop;
                //lcd_clear();//prepare screen for next state        
                //printf("TIRE DROP");
                //setMotorSpeeds(0, true, true, 0);
				break;
			case tireDrop:
                //send command to arduino via i2c asking if the tire-drop is complete;
				isDoneDrop = requestIsTireDropDone();
                //isDoneDrop is given true or false in communication over I2C;
				if (isDoneDrop){
					tiresToDrop -- ;
                    tiresDeployedOnPole++;
					if (tiresToDrop == 0){
						currentOperationState = leavePole;
                                            //reset pole data for next pole:
                        tiresOnPole = 0;
                        totalTiresSupplied += tiresDeployedOnPole;//???? QUESTION IF THIS IS THE RIGHT ALGORITHM OR IF THIS SHOULD BE TIRES THAT MAKE IT TO POLE
                        tiresDeployedOnPole = 0;
					} else {
                        //send command to Arduino via I2C asking for a tire drop for the next tire
						sendArduinoTireDropRequest();	
					}
				}


                
				break;
			case courseAdjustment:
				//adjustCourse();//improve
                currentOperationState = moveForward;
				break;
            case leavePole:
                //move forward a bit to leave tire without disrupting break-beams
                lcd_clear();
                lcd_home();
                printf("Leaving Pole");
                int startedDistA = motorADistance;
                int startedDistB = motorBDistance;
                /*while(millisecondsMeasured < timeInOperation && abs(motorADistance - startedDistA) < 3000){
                    errorCorrectedMove(30, 1, startedDistA, startedDistB, 0.1);
                }*/
                opErrorCorrectionDegrees(30, 1, 0.1, 3000, timeInOperation);
                setMotorSpeeds(0, true, true, 0);
                opDelay(4000, timeInOperation);        
                //set the state to continue to next pole
                motorAStartDistance = 0;//start distance must be reset before the next movement forward
                motorBStartDistance = 0;
                currentOperationState = moveForward;
                
                //
                topLaserState = PORTBbits.RB4;//set top laser state variable to the current state of the laser
                topLaserStatePrev = PORTBbits.RB4;
                bottomLaserState = PORTBbits.RB5;//set top laser state variable to the current state of the laser
                bottomLaserStatePrev = PORTBbits.RB5;
                
                break;
			case returnHome://to be worked on after evaluation
				//returnHome();//improve
				currentOperationState = complete;
				break;
			case complete:
                //FIX this should update logs constantly
                sendArduinoAbortOperationMessage();
                for (int i = 0; i < 48; i++){
                    logs[0][i] = 3;//index 0 means the justRan log
                }
				setMotorSpeeds(0, true, true, 0);
                LATAbits.LATA4 = 0; //enable the keypad once again now that operation is over
				return;
                break;
        }
		
		//UPDATING THE STATE BELOW
        /*
		checkForPolesInWay(&leftRangeFinder, &rightRangeFinder);//check the laser range-finder
		if (leftRangeFinder < minimumSafeDistanceToPole || rightRangeFinder < minimumSafeDistanceToPole){
			currentOperationState = courseAdjustment;//DETAILS LEFT????
		}*/
		/*if (motorADistance > fourMetreEquivalent){
			if (timeToGoToBase(motorADistance, currentTime) == true){
				currentOperationState = return;
			} else {
				current = complete;
			}
		}*/
		//check for the various interrupt conditions
		if (millisecondsMeasured >= timeInOperation){
			currentOperationState = complete;
            //also send ABORT message to arduino
		}
		switch (currentOperationState){//FINISH DETAILS!!!!!
            case moveForward: //moving the robot to the next pole	   
                /*if (bottomBreakBeamTriggeredChange || topBreakBeamTriggeredChange){
					bottomBreakBeamTriggeredChange = 0;
                    topBreakBeamTriggeredChange = 0;
					currentOperationState = poleFinding;
				}*/
                if (topLaserStatePrev != topLaserState || bottomLaserStatePrev != bottomLaserState){
                    if (topLaserStatePrev != topLaserState){
                        badCount++;
                    } 
                    if (bottomLaserStatePrev != bottomLaserState){
                        badCountBot++;
                    }
                    currentOperationState = poleFinding;//UNCOMMENT THIS FOR TESTING ARDUINO
                }//do same for bottom laser as well
				break;
            case poleFinding: //finding out the number of tires on the pole and positioning the robot by the pole
				break;
			case tireDrop:
				break;
			case courseAdjustment:
				break;
            case leavePole:
                break;
			case returnHome:
				break;
			case complete:
				break;
        }
		
        
    }
    //RBIE = 0; //disable the on change interrupts for the laser break beams


    return;
}
/////////////////////////////
//START OF EEPROM SAMPLE CODE
// read and write eeprom of PIC18F2550 (and 18F2455, 18F4455, 18F4550)
// EEPROM size is 256 bytes
// (c) Raphael Wimmer. Licensed under GNU GPL v2 or higher

#pragma stack 0x300 0xff // set 64 byte stack at 0x300, needed by sdcc


void ee_write_byte(unsigned char address, unsigned char *_data){

    EEDATA = *_data;
    EEADR = address;
    // start write sequence as described in datasheet, page 91
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1; // enable writes to data EEPROM
    INTCONbits.GIE = 0;  // disable interrupts
    EECON2 = 0x55;
    EECON2 = 0x0AA;
    EECON1bits.WR = 1;   // start writing
    do {
        ClrWdt();
        } while(EECON1bits.WR);                            // occupato ? 
    EECON1bits.WREN=0;                                // disabilita write
    
}

void ee_read_byte(unsigned char address, unsigned char *_data){
    EEADR = address;
    EECON1bits.CFGS = 0;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    *_data = EEDATA;
}
/*
void initUsart()
{
    usart_open(    // Use USART library to initialise the hardware
            USART_TX_INT_OFF
            & USART_RX_INT_OFF
            & USART_BRGH_HIGH
            & USART_ASYNCH_MODE
            & USART_EIGHT_BIT,
            10                      // '10' = 115200 Baud with 20 MHz oscillator and BRGH=1
            );
    stdout = STREAM_USART;
}



// very simple example. use on an erased eeprom
*/
/////////END OF EEPROM SAMPLE CODE
//////////////////////////////////
 
const char happynewyear[7] = {
    00, // 45 Seconds 
    55, // 59 Minutes
    14, // 24 hour mode, set to 23:00
    02, // Sunday
    19, // 3rd
    02, // february
    19  // 2019
};

/** @brief Writes the happynewyear array to the RTC memory */
void rtc_set_time(void){
    I2C_Master_Start(); // Start condition
    I2C_Master_Write(0b11010000); //7 bit RTC address + Write
    I2C_Master_Write(0x00); // Set memory pointer to seconds
    
    // Write array
    for(char i=0; i < 7; i++){
        I2C_Master_Write(happynewyear[i]);
    }
    
    I2C_Master_Stop(); //Stop condition
}


void robotInit(void){
    //initialize motor encoder stuff:
    EncoderInit();
    // RD2 is the character LCD RS
    // RD3 is the character LCD enable (E)
    // RD4-RD7 are character LCD data lines
    LATD = 0x00;
    TRISD = 0x00;
    
    // RB1, RB4, RB5, RB6, RB7 as inputs (for keypad)
    LATB = 0x00;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB6 = 1;
    TRISBbits.TRISB7 = 1;
    
    //RA4 is an output for the kpd
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 0; // leave keypad kpd bit off so keypad is functional
    
    // Initialize I2C Master with 100 kHz clock
    I2C_Master_Init(100000);
    // Set the time in the RTC. To see the RTC keep time, comment this line out
    // after programming the PIC directly before with this line included
   //rtc_set_time();
    //unsigned char* time = (unsigned char*) malloc(7*sizeof(unsigned char));
    
    // Set all A/D ports to digital (pg. 222)
    ADCON1 = 0b00001111;
    
    // Enable RB1 (keypad data available) interrupt
    INT1IE = 1;
    
    // Initialize LCD
    initLCD();
    
    // Enable interrupts
    ei();
    
    //Turn off cursor:
    //lcd_display_control(true, ) nvm LOL will do this in lcd.c
    //return time;
    return;
}

void replaceOldLog(){
    for (int i = 0; i< 48; i++){
        logs[4][i] = logs[0][i];//index 0 is the justran log and index 4 is the most recent log to be written to eeprom
    }
    return;
}

void main() {
    robotInit();
    
    //unsigned char* time = robotInit();
    unsigned char time[7]; // Create a byte array to hold time read from RTC
    
    //logs format
    //operation time, tires supplied, number of poles, (tires deployed on pole, tires on pole after op, distance to pole) * the pole number
    
    // Main loop
    enum robotStates {standby, operation, operationComplete, selectLog, displayLog};//this is a list of the possible states of the robot
    enum robotStates currentRobotState = standby; 
    enum logStates {justRan,one,two,three,four};//one is the most recent log
    enum logStates currentLogState = one; 
    
    unsigned long tick = 0ul;
    //const char* msg = msg1;
    while(1){
            if (tick%10ul==0){//if 1 second has passed then increase the current time
                        // Reset RTC memory pointer
                I2C_Master_Start(); // Start condition
                I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
                I2C_Master_Write(0x00); // Set memory pointer to seconds
                I2C_Master_Stop(); // Stop condition

                // Read current time
                I2C_Master_Start(); // Start condition
                I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
                for(unsigned char i = 0; i < 6; i++){
                    time[i] = I2C_Master_Read(ACK); // Read with ACK to continue reading
                }
                time[6] = I2C_Master_Read(NACK); // Final Read with NACK
                I2C_Master_Stop(); // Stop condition

                // Print received data on LCD
                //lcd_home();
                //printf("%d/%d/%d", time[6],time[5],time[4]); // Print date in YY/MM/DD
                //lcd_set_ddram_addr(LCD_LINE2_ADDR);
                //printf("%d:%d:%d", time[2],time[1],time[0]); // HH:MM:SS

            }
        
        switch (currentRobotState){
            case standby:
                //display standby screen
                lcd_home();
                
                printf("%s", "WELCOME ");
                printf("%02x/%02x/%02x", time[6],time[5],time[4]);//ADDED HERE
                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                printf("%02x:%02x:%02x", time[2],time[1],time[0]); // HH:MM:SS
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("%s","(1) START");
                lcd_set_ddram_addr(LCD_LINE4_ADDR);
                printf("%s","(2) VIEW LOGS");
                break;
            case operation:
                //display operation screen
                lcd_home();
                printf("%s", "OPERATION IN");
                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                printf("%s","PROGRESS");
                doOperation();
                lcd_clear();//prepare screen for next state
                currentRobotState = operationComplete;//change state after finishing operation
                
                break;
            case operationComplete:
                
                //display completion screen
                lcd_home();
                printf("%s", "OP. COMPLETE");
                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                printf("%s","(1) VIEW LOG");
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("%s","(2) QUIT DO NOT");
                lcd_set_ddram_addr(LCD_LINE4_ADDR);
                printf("%s","SAVE LOG");
                currentLogState = justRan;
                break;
            case selectLog:
                //display selectLog screen
                lcd_home();
                printf("%s", "SELECT LOG");
                lcd_set_ddram_addr(LCD_LINE2_ADDR);
                printf("%s","(1 IS OLDEST)");
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("%s","(1) (2) (3) (4)");
                break;
            case displayLog:
                if (currentLogState == justRan){
                    replaceOldLog();
                    currentLogState = four;
                }
                doDisplayLog(currentLogState);
                lcd_clear(); //prepare screen for next state
                currentRobotState = standby; //change state after finishing displaying logs
                break;
        }
        
        //next state logic based on current state and and button pressed
        if(keyPressed){
            
            
            keyPressed = false; // Clear the flag
            
            // Write key press data to bottom line of LCD
            unsigned char keypress = (PORTB & 0xF0) >> 4;
            char keyValue = keys[keypress];
            lcd_clear(); //clear lcd for next state displayed to be correct
            switch (currentRobotState){
                case standby:
                    if (keyValue == '1'){
                        currentRobotState = operation;
                    } else if (keyValue == '2'){
                        currentRobotState = selectLog;
                    } 
                    break;
                case operation:
                    break;
                case operationComplete:
                    if (keyValue == '1'){
                        currentRobotState = displayLog;
                    } else if (keyValue == '2'){
                        currentRobotState = standby;
                    }
                    break;
                case selectLog:
                    if (keyValue == '1'){
                        currentLogState = one;
                    } else if (keyValue == '2'){
                        currentLogState = two;
                    } else if (keyValue == '3'){
                        currentLogState = three;
                    } else if (keyValue == '4'){
                        currentLogState = '4';
                    } else {
                        break;
                    }
                    currentRobotState = displayLog;
                    break;
                case displayLog:
                    //this case is taken care of by the doDisplayLog() function
                    break;
            }
            /*lcd_set_ddram_addr(LCD_LINE2_ADDR);
            putch(keys[keypress]);*/
        }
        /*
        if(tick % 1000 == 0){
            lcd_home();
            printf("%s", msg);
            msg = (msg == msg1) ? msg2 : msg1; // Alternate the message
        }
        */
        
        tick++;
        __delay_ms(1);
    }
}

/**
 * @brief Any time an interrupt is generated, the microcontroller will execute
 *        this function (as long as interrupts are enabled). Any interrupts
 *        which are enabled need to be handled in this function, otherwise
 *        unexpected behavior will arise, perhaps even causing the PIC to reset
 *        (you AT LEAST need to clear the flag for each interrupt which is
 *        enabled!)
 */
void __interrupt() interruptHandler(void){
    // Interrupt on change handler for RB1
    if(INT1IF){
        // Notice how we keep the interrupt processing very short by simply
        // setting a "flag" which the main program loop checks
        keyPressed = true;
        INT1IF = 0; // Clear interrupt flag bit to signify it's been handled
    } else if (TMR0IF) {   //Check if Timer0 has caused the interrupt; if it has then increase secondsMeasured
    INTCONbits.TMR0IF = 0;      //CLEAR FLAG SO IT CAN BE TRIGGERED AGAIN
    TMR0H = TIMER0_COUNTS_PER_HALF_SECOND >> 8;
    TMR0L = TIMER0_COUNTS_PER_HALF_SECOND;
    millisecondsMeasured++;           //increase the timer count
   } else if (INT2IF){//motor B encoder RB2
       INT2IF = 0;
       updateMotorBEncoder();
   } else if (INT0IF){//motor A encoder RB0
       INT0IF = 0;
       updateMotorAEncoder();
   }  /*else if (RBIF){//if it is the laser break beam changing state
       PORTBbits.RB4;
       RBIF = 0;
       topBreakBeamTriggeredChange = 1;
   }*/
}

void mainEEPROM(void){
    robotInit();
        lcd_clear();
    lcd_home();
    char save_me = 'x';
    char from_eeprom;

    //initUsart();
     robotInit();
    printf("EEPROM-Demon");
    ee_read_byte(0x00, &from_eeprom);
    printf("read: %cn", from_eeprom);
    
    ee_write_byte(0x00, &save_me);
    printf("wrote: %cn", save_me);
    
    ee_read_byte(0x00, &from_eeprom);
    printf("read: %cnn", from_eeprom);
}
