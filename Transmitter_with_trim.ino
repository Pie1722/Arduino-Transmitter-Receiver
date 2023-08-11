// Arduino Transmitter with Trim 
  
 #include <SPI.h>
 #include <nRF24L01.h>
 #include <RF24.h>
 #include <EEPROM.h>
 #include <Wire.h>
 #include <avr/sleep.h> 
 #include <avr/power.h>

 // Define the time (in milliseconds) after which the Arduino should go to sleep
 const unsigned long INACTIVITY_TIMEOUT = 600000; // 10 minutes

 unsigned long lastActivityTime = 0; // Variable to store the time of the last joystick activity

 // communication pipe is defined for the NRF24L01 module. The radio object is created to interact with the NRF24L01 module using the pins 9 (CE) and 10 (CSN).
 const uint64_t pipeOut = 000322;        // NOTE: The same as in the receiver 000322  
 RF24 radio(9, 10);                      // nRF24L01 (CE, CSN)

/*Defines the analog pins for reading joystick X and Y axis values. joystickThreshold sets the minimum change required to consider the joystick as active. 
  tempRaw stores the raw temperature reading from a sensor, and joystickActive keeps track of whether the joystick is actively being used. */
 const int joystickPinX = A0;    // Joystick X-axis pin
 const int joystickPinY = A1;    // Joystick Y-axis pin
 const int joystickThreshold = 50; // Minimum change required to wake up

 int16_t tempRaw;               //16 bit integer used to store raw temp data from MPU6050
 //boolean (true/false) value.
 bool joystickActive = false;  // Initializing that the joystick is not active

 //Defines the pin numbers for buttons used to adjust trim values. 
 #define trimbut_1 1                      // Trim button 1 / Pin D1
 #define trimbut_2 2                      // Trim button 2 / Pin D2
 #define trimbut_3 3                      // Trim button 3 / Pin D3
 #define trimbut_4 4                      // Trim button 4 / Pin D4
 #define trimbut_5 5                      // Trim button 5 / Pin D5
 #define trimbut_6 6                      // Trim button 6 / Pin D6

 const int MPU = 0x68;                   // MPU6050 I2C address
 float AccX, AccY, AccZ;          //These variables are used to store the accelerometer readings for the X, Y, and Z axes.
 float GyroX, GyroY, GyroZ;       //These variables are used to store the gyroscope readings for the X, Y, and Z axes.
 float accAngleX, accAngleY, gyroAngleX, gyroAngleY;   //These variables are used to store calculated angle values based on accelerometer and gyroscope data.
 float angleX, angleY;
 float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY;  //Variables store error values that are calculated during setup to correct for errors in accelerometer and gyroscope readings.
 
 /* ElapsedTime stores the time duration between successive gyro readings. It's of type float and represents time in seconds.
    CurrentTime and previousTime store the current and previous time in milliseconds. They help calculate elapsedTime.
    c is an integer variable that is used as a counter in certain sections of the code, like during error calculation. */
 float elapsedTime, currentTime, previousTime;
 int c = 0;
 
 int tvalue1 = EEPROM.read(1) * 4;        // Reading trim values from Eeprom  
 int tvalue2 = EEPROM.read(3) * 4;        
 int tvalue3 = EEPROM.read(5) * 4;        

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
  struct Signal {  //Defines a structure named Signal that represents the control signals to be transmitted
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
  //The byte data type is an 8-bit unsigned integer that can represent values from 0 to 255.
};

  Signal data;   //The line creates an instance of the Signal structure of above named data
  
  void ResetData() {                 // If signal lost then the default position of all axis 
  data.throttle = 12;                      
  data.pitch = 127;
  data.roll = 127;
  data.yaw = 127;
  data.aux1 = 0;                         
  data.aux2 = 0;
  data.aux3 = 0;
  data.aux4 = 0;
  data.aux5 = 0;
  data.aux6 = 0;
}

  void setup() {

  Serial.begin(9600);
  
  // Initialize interface to the MPU6050
  initialize_MPU6050();

  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error(); 

  lastActivityTime = millis(); // Initialize lastActivityTime

  // Define the radio communication                                         
  radio.begin();  //initializes the nRF24L01 radio module
  Serial.println("Radio COM Started");
  radio.openWritingPipe(pipeOut);  //This sets the pipe address that the radio module will use to transmit data
  radio.setAutoAck(false);         //This disables the automatic acknowledgment feature.
  radio.setDataRate(RF24_250KBPS);       // The lowest data rate value for more stable communication  
  radio.setPALevel(RF24_PA_MAX);         // This sets the power amplifier (PA) level to maximum.
  radio.stopListening();                 // This line instructs the radio module to stop listening for incoming data.
  ResetData();                // Used to reset all values if the connnection is lost

  // Activate the internal pull-up resistors
  pinMode(trimbut_1, INPUT_PULLUP);
  pinMode(trimbut_2, INPUT_PULLUP);
  pinMode(trimbut_3, INPUT_PULLUP); 
  pinMode(trimbut_4, INPUT_PULLUP);
  pinMode(trimbut_5, INPUT_PULLUP); 
  pinMode(trimbut_6, INPUT_PULLUP);
  //Defines analog pin as input
  pinMode(joystickPinX, INPUT);
  pinMode(joystickPinY, INPUT);

  tvalue1= EEPROM.read(1) * 4;
  tvalue2= EEPROM.read(3) * 4;
  tvalue3= EEPROM.read(5) * 4;
  //These line reads a value from the EEPROM memory from respected address and stores at respected variables
}


 void enterSleepMode() {
   sleep_enable();  // Enable sleep
   set_sleep_mode(SLEEP_MODE_IDLE);  // Set sleep mode to Idle
   sleep_mode();    // Enter sleep mode
   sleep_disable(); // Disable sleep after waking up
}

 void checkJoystickActivity() {                //analogRead reads values from 0 to 1023
   int joystickX = analogRead(joystickPinX);   //Represents X-Axis of Joystick
   int joystickY = analogRead(joystickPinY);   //Represents Y-Axis of Joystick

//This condition compares the absolute difference between the current joystick analog value and the center value (512, which represents the neutral position) with the joystickThreshold.
     if (abs(joystickX - 512) > joystickThreshold || abs(joystickY - 512) > joystickThreshold) {  
       joystickActive = true;  // If the condition meets them the Joystick is ACTIVE or else it's NOT ACTIVE
      } else {
         joystickActive = false;
        }
}

/* Border_Map that takes five integer arguments:
 * val (the input value to be mapped)
 * lower (the lower bound of the input range)
 * middle (a value that defines the split point between two mapping ranges)
 * upper (the upper bound of the input range)
 * reverse (a boolean indicating whether to reverse the output range)
 */
  int Border_Map(int val, int lower, int middle, int upper, bool reverse) 
{  
  val = constrain(val, lower, upper);
  //The constrain function is used here to ensure that the input value val is within the specified range [lower, upper]. If val is outside this range, it is clipped to the closest bound.
  if ( val < middle ) 
  //If val is less than middle, it means that the input value is in the lower range. In this case, the map function is used to linearly map the input value from the lower to middle range to an output range of [0, 128]
  val = map(val, lower, middle, 0, 128);
  //If val is greater than or equal to middle, it means that the input value is in the upper range. In this case, the map function is used to linearly map the input value from the middle to upper range to an output range of [128, 255].
  else
  val = map(val, middle, upper, 128, 255);
  // If the reverse argument is true, the output value is further transformed by subtracting it from 255.
  //This effectively reverses the output range, giving a higher output value for lower input values and vice versa.
  return ( reverse ? 255 - val : val );
}

  void loop()
{ 
  
  checkJoystickActivity();
  unsigned long currentTime = millis();

  if (joystickActive) {
        lastActivityTime = currentTime; // Update last activity time

  readTemp();
 
// Trims and Limiting trim values  

  if(digitalRead(trimbut_1)==LOW and tvalue1 < 630) {
    tvalue1=tvalue1+15;
    EEPROM.write(1,tvalue1/4); 
    delay (10);

  }   
  
  if(digitalRead(trimbut_2)==LOW and tvalue1 > 280){
    tvalue1=tvalue1-15;
    EEPROM.write(1,tvalue1/4);
    delay (10);
  } 
  
  if(digitalRead(trimbut_3)==LOW and tvalue2 < 630) {
    tvalue2=tvalue2+15;
    EEPROM.write(3,tvalue2/4);
    delay (10);
  } 
    
  if(digitalRead(trimbut_4)==LOW and tvalue2 > 280){
    tvalue2=tvalue2-15;
    EEPROM.write(3,tvalue2/4);
    delay (10);
  } 
  
  if(digitalRead(trimbut_5)==LOW and tvalue3 < 630) {
     tvalue3=tvalue3+15;
     EEPROM.write(5,tvalue3/4);
     delay (10);
  }  
   
  if(digitalRead(trimbut_6)==LOW and tvalue3 > 280){
    tvalue3=tvalue3-15;
    EEPROM.write(5,tvalue3/4);
    delay (10);
  }

  
// Control Stick Calibration for channels  

  data.roll = Border_Map( analogRead(A3), 0, tvalue1, 1023, true );      // "true" or "false" for signal direction | Alieron
  data.pitch = Border_Map( analogRead(A2), 0, tvalue2, 1023, true );     // Elevator    
  data.throttle = Border_Map( analogRead(A1),570, 800, 1023, false );    // For Single side ESC | 
  // data.throttle = Border_Map( analogRead(A1),0, 512, 1023, false );   // For Bidirectional ESC | 
  data.yaw = Border_Map( analogRead(A0), 0, tvalue3, 1023, true );       // Rudder          
  data.aux1 = Border_Map( analogRead(A4), 0, 512, 1023, true );                 
  data.aux2 = Border_Map( analogRead(A5), 0, 512, 1023, true );                 
  data.aux3 = Border_Map( analogRead(A6), 0, 512, 1023, true );
  data.aux4 = Border_Map( analogRead(A7), 0, 512, 1023, true );
  data.aux5 = digitalRead(7);
  data.aux6 = digitalRead(8);

  // If D0 is high 
  if (digitalRead(0) == 1) {
    read_IMU();    // Use MPU6050 instead of Joystick for controling left, right, forward and backward movements
  }
  
   radio.write(&data, sizeof(Signal)); 
    
    } else {
        // If joystick is inactive for INACTIVITY_TIMEOUT, enter sleep mode
        if (currentTime - lastActivityTime >= INACTIVITY_TIMEOUT) {
            Serial.println("Going to sleep due to Inactivity");
            enterSleepMode();
        }
    }
}

 void initialize_MPU6050() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Configure Accelerometer
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);
  // Configure TEMP
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);                   // PWR_MGMT_1 register
  Wire.write(0);                      // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gury data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

// For each axis (X, Y, and Z), two bytes are read from the sensor and then combined using bit-shifting (<< 8) and bitwise OR operations to form a 16-bit value.
// The combined 16-bit value is divided by 4096.0 to convert it into an acceleration value in g-units (gravity units).
    AccX = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
// Measure the average error or bias present in the accelerometer readings due to sensor imperfections.
// These error values can be used later to compensate for inaccuracies when calculating the orientation of the sensor based on accelerometer data.
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 4, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 32.8);
    GyroErrorY = GyroErrorY + (GyroY / 32.8);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
}

void read_IMU() {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-8g, we need to divide the raw values by 4096, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value

  // Calculating angle values using
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) + 1.15; // AccErrorX ~(-1.15) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - 0.52; // AccErrorX ~(0.5)

  // === Read gyro data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;   // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 4, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 32.8; // For a 1000dps range we have to divide first the raw value by 32.8, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 32.8;
  GyroX = GyroX + 1.85; //// GyroErrorX ~(-1.85)
  GyroY = GyroY - 0.15; // GyroErrorY ~(0.15)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = GyroX * elapsedTime;
  gyroAngleY = GyroY * elapsedTime;

  // Complementary filter - combine acceleromter and gyro angle values
  angleX = 0.98 * (angleX + gyroAngleX) + 0.02 * accAngleX;
  angleY = 0.98 * (angleY + gyroAngleY) + 0.02 * accAngleY;
  // Map the angle values from -90deg to +90 deg into values from 0 to 255, like the values we are getting from the Joystick
  data.throttle = map(angleX, -90, +90, 255, 0);
  data.roll = map(angleY, -90, +90, 0, 255);
}

void readTemp()  {

  Wire.beginTransmission(MPU);
  Wire.write(0x41);  // Temp register address
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU, 2, true);
  tempRaw = Wire.read() << 8 | Wire.read();  // Combine the two bytes
  
  float temperature = (tempRaw / 340.0) + 36.53;  // Conversion formula from datasheet
  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  
    if (temperature >= 100.0) {
  Serial.println("Entering sleep mode.");
  enterSleepMode();
  }

}
