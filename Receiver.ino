/* By
 * HYPERARX
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
struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte tSwitch3;
  byte tSwitch4;
};
Data_Package data; //Create a variable with the above structure
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
  steering = data.j2PotX;
  throttle = data.j1PotY;
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
  data.j1PotX = 127.5;
  data.j1PotY = 127.5;
  data.j2PotX = 127.5;
  data.j2PotY = 127.5;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 1;
  data.pot2 = 1;
  data.tSwitch1 = 1;
  data.tSwitch2 = 1;
  data.button1 = 1;
  data.button2 = 1;
  data.tSwitch3 = 1;
  data.tSwitch4 = 1;
}
