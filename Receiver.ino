/* By
 * Pie_1722
 * aravindvallab@protonmail.com
 * Arduino Pro Mini Receiver (RC CAR) 
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define in3 5  // D5 - CH2 - PWM output
#define in4 6  // D6 - CH3 - PWM output
#define in1 7  // D7 - CH4
#define in2 8  // D8 - CH5

RF24 radio(3, 2);   // nRF24L01 (CE, CSN)
const uint64_t pipeIn = 0xE9E8F0F0E1LL;
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;

// Max size of this struct is 32 bytes
struct Signal {
  byte throttle;
  byte pitch;
  byte roll;
  byte yaw;
  byte aux1;
  byte aux2;
  byte aux3;
  byte aux4;
  byte aux5;
  byte aux6 ; 
};
Signal data; //Create a variable with the above structure

int  steering, throttle;
int motorSpeedA = 0;
int motorSpeedB = 0;

void setup() {
  Serial.begin(9600);
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  radio.begin();
  radio.openReadingPipe(0, pipeIn);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //  Set the module as receiver
  resetData();
}
void loop() {
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone jas a throttle up, if we lose connection it can keep flying away if we dont reset the function
  }
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data
  }
  // Parse the data from the Joystic 1 to the steering and throttle variables
  steering = data.roll;
  throttle = data.throttle;
// Throttle used for forward and backward control
  if (throttle < 110) {
    // Convert the declining throttle readings for going backward from 110 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedB = map(throttle, 110, 0, 0, 255);
    // Set Motor B backward
    analogWrite(in3, motorSpeedB);
    digitalWrite(in4, LOW);
  }
  else if (throttle > 140) {
    // Convert the increasing throttle readings for going forward from 140 to 255 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedB = map(throttle, 140, 255, 0, 255);
    // Set Motor B forward
    digitalWrite(in3, LOW);
    analogWrite(in4, motorSpeedB);
  }
  // If joystick stays in middle the motors are not moving
  else {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, HIGH);
  }
  
// steering used for left and right control
  if (steering < 110) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  if (steering > 140) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  // If joystick stays in middle the motors are not moving
  else {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  }
}
void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.yaw = 127.5;
  data.throttle = 127.5;
  data.roll = 127.5;
  data.pitch = 127.5;
  data.aux1 = 0;                                              // Define the inicial value of each data input. 
  data.aux2 = 0;
  data.aux3 = 0;
  data.aux4 = 0;
  data.aux5 = 0;
  data.aux6 = 0;
}
