#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

int wheel1;
int wheel1a;
int wheel2;
int wheel2a;

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t menuDown;  
    int16_t Select;    
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1;
    int16_t toggle2;
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

int RLR = 0;
float RFB = 0;
float RFBa = 0;
int RT = 0;
int LLR = 0;
int LFB = 0;
int LT = 0;
int LTa = 0;

float cor;
float insideAngle;
float outsideAngle;
float insideAngleOutput;
float outsideAngleOutput;
float insideVel;
float outsideVel;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer

int servo1Offset = 1430;
int servo2Offset = 1375;
int servo3Offset = 1470;
int servo4Offset = 1500;

int servo45 = 400;

void setup() {

    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
   
    // initialize serial communication
    Serial.begin(115200);
    
    radio.begin();
    radio.openWritingPipe(addresses[0]); // 00002
    radio.openReadingPipe(1, addresses[1]); // 00001
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();

    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);

    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(12, OUTPUT);

    servo1.attach(32);
    servo2.attach(34);
    servo3.attach(36);
    servo4.attach(38);

    servo1.writeMicroseconds(servo1Offset);
    servo2.writeMicroseconds(servo2Offset);
    servo3.writeMicroseconds(servo3Offset);
    servo4.writeMicroseconds(servo4Offset);      


    
}   // end of setup

// ********************* MAIN LOOP *******************************

void loop() {  

      
        currentMillis = millis();
        if (currentMillis - previousMillis >= 20) {  // start timed event
          
            previousMillis = currentMillis;


            // check for radio data
            if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));   
            }             

            else {
              Serial.println("no data");
            }

            // threshold remote data
            // some are reversed based on stick wiring in remote
            RFB = (thresholdStick(mydata_remote.RFB))/2;   
            RLR = (thresholdStick(mydata_remote.RLR))/2;
            LT = (thresholdStick(mydata_remote.LT))/2;
            LLR = (thresholdStick(mydata_remote.LLR))/2; 


            if (mydata_remote.toggleTop == 1) {                       // rotate on the spot
                  
                servo1.writeMicroseconds(servo1Offset + servo45);
                servo2.writeMicroseconds(servo2Offset - servo45);
                servo3.writeMicroseconds(servo3Offset - servo45);
                servo4.writeMicroseconds(servo4Offset + servo45); 

                LT = LT / 1.5;        // scale driving stick some more

               if (LT >=0) {
                  LTa = abs(LT);
                  analogWrite(4, LTa);       // wheel 1
                  analogWrite(3, 0);
                  analogWrite(6, LTa);       // wheel 2
                  analogWrite(5, 0);
                  analogWrite(9, 0);       // wheel 3
                  analogWrite(10, LTa);
                  analogWrite(11, 0);       // wheel 4
                  analogWrite(12, LTa);
                  
                }

                else if (LT <=0) {
                  LTa = abs(LT);
                  analogWrite(4, 0);          // wheel 1
                  analogWrite(3, LTa);
                  analogWrite(6, 0);          // wheel 2
                  analogWrite(5, LTa);
                  analogWrite(9, LTa);       // wheel 3
                  analogWrite(10, 0);
                  analogWrite(11, LTa);       // wheel 4
                  analogWrite(12, 0);
                  
                }                         
            }

            else {                                                  // swerve steering

                // trig calcs for Ackermann steering
                
                cor = 1000-abs(LLR*3.5);                            // scale centre of rotation from stick

                insideAngle = atan(104 / (cor - 114));              // calc angle in radians
                insideAngle = insideAngle * (180/PI);               // convert to degrees    

                outsideAngle = atan(104 / (cor + 114));
                outsideAngle = outsideAngle * (180/PI);

                outsideAngle = outsideAngle * 9;                    // convert to servo ms
                insideAngle = insideAngle * 9;                      // convert to servo ms

                if (LLR == 0) {
                  insideAngleOutput = 0;                            // steer straight when the stick is in the middle
                  outsideAngleOutput = 0;
                } 
                else if (LLR >=0) {
                  insideAngleOutput = outsideAngle*-1;              // invert angles when we steer the other way
                  outsideAngleOutput = insideAngle*-1;
                }
                else if (LLR <=0) {
                  insideAngleOutput = insideAngle;                  // angles can be used for the default side of the vehicle as they are
                  outsideAngleOutput = outsideAngle;
                }

                Serial.print(insideAngle);
                Serial.print(" , ");
                Serial.println(outsideAngle);
 
           
                servo1.writeMicroseconds(servo1Offset + (LT * 3.8) - insideAngleOutput);        // write out the servo positions
                servo2.writeMicroseconds(servo2Offset + (LT * 3.8) + insideAngleOutput);        // use the left twist stick for swerve steering
                servo3.writeMicroseconds(servo3Offset + (LT * 3.8) - outsideAngleOutput);       // and use the angle calc for Ackermann steering.
                servo4.writeMicroseconds(servo4Offset + (LT * 3.8) + outsideAngleOutput); 



                insideVel = float ((RFB/100)*outsideAngle);
                outsideVel = float ((RFB/100)*insideAngle);
                insideVel = constrain(insideVel,-255,255);
                outsideVel = constrain(outsideVel,-255,255);



                // write to motors 
               if (LLR == 0) {

                  // *** straight line ***

                  RFB = RFB * 0.5;          // scale throttle a little more
               
                  if (RFB > 0) {
                    RFBa = abs(RFB);
                    analogWrite(3, RFBa);       // wheel 1
                    analogWrite(4, 0);
                    analogWrite(5, RFBa);       // wheel 2
                    analogWrite(6, 0);
                    analogWrite(9, 0);       // wheel 3
                    analogWrite(10, RFBa);
                    analogWrite(11, 0);       // wheel 4
                    analogWrite(12, RFBa);                  
                  }
                  // backwards  
                  else if (RFB < 0) {
                    RFBa = abs(RFB);
                    analogWrite(3, 0);          // wheel 1
                    analogWrite(4, RFBa);
                    analogWrite(5, 0);          // wheel 2
                    analogWrite(6, RFBa);
                    analogWrite(9, RFBa);       // wheel 3
                    analogWrite(10, 0);
                    analogWrite(11, RFBa);       // wheel 4
                    analogWrite(12, 0);                  
                  }
                // stop
                    else {
                      analogWrite(3, 0);          // wheel 1
                      analogWrite(4, 0);
                      analogWrite(5, 0);          // wheel 2
                      analogWrite(6, 0);
                      analogWrite(9, 0);       // wheel 3
                      analogWrite(10, 0);
                      analogWrite(11, 0);       // wheel 4
                      analogWrite(12, 0);
                    }
              }


              if (LLR > 0) {  // *** steer right ***                     

                  // forwards
               
                  if (RFB > 0) {
                    outsideVel = abs(outsideVel);
                    insideVel = abs(insideVel);
                    analogWrite(3, outsideVel);       // wheel 1
                    analogWrite(4, 0);
                    analogWrite(5, outsideVel);       // wheel 2
                    analogWrite(6, 0);
                    analogWrite(9, 0);       // wheel 3
                    analogWrite(10, insideVel);
                    analogWrite(11, 0);       // wheel 4
                    analogWrite(12, insideVel);                  
                  }
                  // backwards  
                  else if (RFB < 0) {
                    outsideVel = abs(outsideVel);
                    insideVel = abs(insideVel);
                    analogWrite(3, 0);          // wheel 1
                    analogWrite(4, outsideVel);
                    analogWrite(5, 0);          // wheel 2
                    analogWrite(6, outsideVel);
                    analogWrite(9, insideVel);       // wheel 3
                    analogWrite(10, 0);
                    analogWrite(11, insideVel);       // wheel 4
                    analogWrite(12, 0);                  
                  }
                // stop
                    else {
                      analogWrite(3, 0);          // wheel 1
                      analogWrite(4, 0);
                      analogWrite(5, 0);          // wheel 2
                      analogWrite(6, 0);
                      analogWrite(9, 0);       // wheel 3
                      analogWrite(10, 0);
                      analogWrite(11, 0);       // wheel 4
                      analogWrite(12, 0);
                    }
              }

              if (LLR <0) {  // *** steer left ***                      

                  // forwards
               
                  if (RFB > 0) {
                    outsideVel = abs(outsideVel);
                    insideVel = abs(insideVel);
                    analogWrite(3, insideVel);       // wheel 1
                    analogWrite(4, 0);
                    analogWrite(5, insideVel);       // wheel 2
                    analogWrite(6, 0);
                    analogWrite(9, 0);       // wheel 3
                    analogWrite(10, outsideVel);
                    analogWrite(11, 0);       // wheel 4
                    analogWrite(12, outsideVel);                  
                  }
                  // backwards  
                  else if (RFB < 0) {
                    outsideVel = abs(outsideVel);
                    insideVel = abs(insideVel);
                    analogWrite(3, 0);          // wheel 1
                    analogWrite(4, insideVel);
                    analogWrite(5, 0);          // wheel 2
                    analogWrite(6, insideVel);
                    analogWrite(9, outsideVel);       // wheel 3
                    analogWrite(10, 0);
                    analogWrite(11, outsideVel);       // wheel 4
                    analogWrite(12, 0);                  
                  }
                // stop
                    else {
                      analogWrite(3, 0);          // wheel 1
                      analogWrite(4, 0);
                      analogWrite(5, 0);          // wheel 2
                      analogWrite(6, 0);
                      analogWrite(9, 0);       // wheel 3
                      analogWrite(10, 0);
                      analogWrite(11, 0);       // wheel 4
                      analogWrite(12, 0);
                    }
              }

            }


                          
                



      
        }     // end of timed loop         

   
}       // end  of main loop
