
// Robot by DAB
// 
// Using Molon Geared Dual Motors
//
#include <NewPing.h>

#define TRIGGER_PIN_LEFT  4  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_LEFT     5  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE_LEFT 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar_LEFT(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE_LEFT); // NewPing setup of pins and maximum distance.

#define TRIGGER_PIN_CENTER  2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_CENTER     3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE_CENTER 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar_CENTER(TRIGGER_PIN_CENTER, ECHO_PIN_CENTER, MAX_DISTANCE_CENTER); // NewPing setup of pins and maximum distance.

#define TRIGGER_PIN_RIGHT  6  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_RIGHT     7  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE_RIGHT 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar_RIGHT(TRIGGER_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE_RIGHT); // NewPing setup of pins and maximum distance.

const long Left_Offset =2;
const long Right_Offset = 2;
const int WheelTread = 18; //wheel tread distance (distance per click) centimeters
const int car_length=50; // length of car, centimeters
const int pinI1=8;//define I1 interface
const int pinI2=11;//define I2 interface 
int speedpinA=9;//enable motor A
const int pinI3=12;//define I3 interface 
const int pinI4=13;//define I4 interface 
int speed_engine =255;//define the speed_engine of motor127
int Distance_CM_CENTER =0; //distance in centimeters
int Distance_CM_LEFT =0; //distance in centimeters
int Distance_CM_RIGHT =0; //distance in centimeters
bool in_motion =false;
int Direction_Decision =0; //what direction? 0=center, 1=left, 2 = right
int Previous_Direction_Decision=0; // what direction were we moving before?
int last_Right=400; //  recent reading
int last_Left=400; //  recent reading
int Distance_before_turn = 0;
int Distance_SoFar = 0;
int Wheel_ReedSwitch = 0;
int pinWheel_ReedSwitch = A1;  // pin used to read clicks from the reed switch (distance)
int speedpinB=10;//enable motor B

//relay variables
const int pinRelay1 = A2;
const int pinRelayCombo2 = A3;
const int brakeDistance = 150;  //centimeters
const int pingDelay = 600; // milliseconds

void setup()
{
  pinMode(pinI1,OUTPUT);
  pinMode(pinI2,OUTPUT);
  pinMode(speedpinA,OUTPUT);
  pinMode(pinI3,OUTPUT);
  pinMode(pinI4,OUTPUT);
  pinMode(speed_engine,OUTPUT);
  Serial.begin(115200); // Open Serial monitor at 115200 baud to see results.
  Serial.println("SERIAL begin");
   
  pinMode(pinRelay1, OUTPUT);
  pinMode(pinRelayCombo2, OUTPUT);
  pinMode(pinWheel_ReedSwitch, INPUT);// pin used to read clicks from the reed switch (distance)
  Serial.println("setup end");

}
 
void forward()
{
     analogWrite(speedpinA,speed_engine);//input a simulation value to set the speed
     analogWrite(speedpinB,speed_engine);
     digitalWrite(pinI3,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI4,LOW);
     digitalWrite(pinI1,LOW);//turn DC Motor A move anticlockwise
     digitalWrite(pinI2,HIGH);
}
void backward()//
{
     analogWrite(speedpinA,speed_engine);//input a simulation value to set the speed
     analogWrite(speedpinB,speed_engine);
     digitalWrite(pinI3,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI4,HIGH);

     digitalWrite(pinI1,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinI2,LOW);
}
void right()//
{
     analogWrite(speedpinA,speed_engine);//input a simulation value to set the speed
     analogWrite(speedpinB,speed_engine);
     digitalWrite(pinI3,HIGH);//turn DC Motor B move clockwise
     digitalWrite(pinI4,LOW);
     digitalWrite(pinI1,HIGH);//turn DC Motor A move clockwise
     digitalWrite(pinI2,LOW);
}
void left()//
{
     analogWrite(speedpinA,speed_engine);//input a simulation value to set the speed
     analogWrite(speedpinB,speed_engine);
     digitalWrite(pinI3,LOW);//turn DC Motor B move anticlockwise
     digitalWrite(pinI4,HIGH);
     digitalWrite(pinI1,LOW);//turn DC Motor A move clockwise
     digitalWrite(pinI2,HIGH);
}
void motorstop()//
{
     digitalWrite(speedpinA,LOW);// Unenble the pin, to stop the motor. this should be done to avid damaging the motor. 
     digitalWrite(speedpinB,LOW);
     delay(1000);
 
}


void DeadEndResolution()
{
    if (Distance_CM_CENTER < brakeDistance){
      motorstop();
      // 2= right
      if  (Direction_Decision = 2){
          left();
      } else {
          right();
      }
      speed_engine = 255;
      backward();
      delay(1000);
      motorstop();
      forward();      
      Serial.println("DEAD END RESOLUTION");

    }
}


void loop()
{

randomSeed(analogRead(5)); // randomize using noise from analog pin 5

long randNumber = random(200, 2500); // it generate random numbers from 200 up

  Serial.println("loop");

  delay(pingDelay);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS_CENTER = sonar_CENTER.ping(); // Send ping, get ping time in microseconds (uS).
  Distance_CM_CENTER = (uS_CENTER / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)
  
  delay(pingDelay);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS_LEFT = sonar_LEFT.ping(); // Send ping, get ping time in microseconds (uS).
  Distance_CM_LEFT = (uS_LEFT / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)

  delay(pingDelay);                      // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  unsigned int uS_RIGHT = sonar_RIGHT.ping(); // Send ping, get ping time in microseconds (uS).
  Distance_CM_RIGHT = (uS_RIGHT / US_ROUNDTRIP_CM); // Convert ping time to distance in cm and print result (0 = outside set distance range)

//adjust zeroes (no range possible) to outermost range
if (Distance_CM_CENTER ==0){
 Distance_CM_CENTER = 400;
}
//adjust zeroes (no range possible) to outermost range
if (Distance_CM_LEFT ==0){
 Distance_CM_LEFT = 400;
}
//adjust zeroes (no range possible) to outermost range
if (Distance_CM_RIGHT ==0){
 Distance_CM_RIGHT = 400;
}

 Serial.print("Distance_CM_CENTER= ");
 Serial.println(Distance_CM_CENTER);
 Serial.print("Distance_CM_LEFT= ");
 Serial.println(Distance_CM_LEFT);
 Serial.print("Distance_CM_RIGHT= ");
 Serial.println(Distance_CM_RIGHT);



Direction_Decision = 0; // 0 = center DEFAULT direction
  if ((Distance_CM_CENTER <= brakeDistance) || (Distance_CM_LEFT <= brakeDistance) || (Distance_CM_RIGHT <= brakeDistance)) {

      if ((Distance_CM_LEFT > Distance_CM_RIGHT) and Previous_Direction_Decision != 1){
            Direction_Decision = 1; // 1 = left
          //  straight_before_turn();
            left();          
            Serial.println("ROTATING LEFT");                    
        } else {
          if ((Distance_CM_LEFT < Distance_CM_RIGHT) and Previous_Direction_Decision != 2){
            Direction_Decision = 2; // 2 = right
          //  straight_before_turn();
            right();          
            Serial.println("ROTATING RIGHT");       
          } else{
            Direction_Decision = 0; // 0= straight            
            }             
    }
        
  }
  
  if (Direction_Decision == 0) {
      
       Serial.println("GO STRAIGHT");
          forward();
          in_motion =true;
           Serial.println("IN MOTION");
      }

      last_Left = Distance_CM_LEFT; // save for later comparison
      last_Right = Distance_CM_RIGHT;
      Previous_Direction_Decision = Direction_Decision;
      delay(randNumber);

      if (Direction_Decision != 0) {
      randNumber = random(1, 10); // it generate random numbers from 200 up
        if (randNumber > 5) {
          backward();
          delay(700);
          motorstop();
        }
      }      
  }
