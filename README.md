# Arduino-Transmitter-Receiver

This project is to make a transmitter and receiver just like commercial transmitter and receiver.
Use transmitter_with_trim as it has more functions. I'm using Arduino Pro Mini therefore there is a limitation in pins so to use SDA communications , comment A4 & A5 to use SDA communication and vice versa to use Analog pins. 

DOWNLOAD CODE - https://github.com/Pie1722/Arduino-Transmitter-Receiver/blob/main/Transmitter_with_trim.ino

# Features

1. Libraries: The program uses various libraries, including SPI, nRF24L01, RF24, EEPROM, Wire (for I2C communication), Adafruit_GFX, and Adafruit_SSD1306 (for controlling an OLED 
   display).

2. OLED Display: The code initializes and uses an OLED display (128x64 pixels) to provide information and user feedback. It can display trim values, auxiliary channel values, and system 
   status.

3. Joystick Input: The program reads analog joystick values (X and Y axes) using two analog pins (A0 and A1). It uses a threshold (joystickThreshold) to detect joystick activity.

4. Trim Adjustment: The code defines six trim buttons (trimbut_1 to trimbut_6) and handles their presses to adjust trim values. Trim values are stored in EEPROM memory and are read 
   during setup.

5. IMU Control: The program interfaces with an MPU6050 IMU sensor to obtain accelerometer and gyroscope data. It calculates angle values (pitch and roll) using a complementary filter 
   and maps these angles to control signals. IMU control can be enabled or disabled using a button press.

6. Radio Communication: It sets up and uses an nRF24L01 radio module for wireless communication. It defines a structure (Signal) to represent control signals and sends these signals 
   wirelessly.

7. Sleep Mode: When the joystick is inactive for a specified time (INACTIVITY_TIMEOUT), the program enters a low-power sleep mode to conserve energy. It also turns off the OLED display 
   and onboard LED in sleep mode.

8. Temperature Monitoring: The program reads the temperature from the MPU6050 sensor and can enter sleep mode if the temperature exceeds a certain threshold.

9. EEPROM Usage: Trim values are read from and written to EEPROM memory to persistently store user adjustments.

10. Error Calibration: During setup, the program calculates error values for both the accelerometer and gyroscope readings to compensate for sensor imperfections.

11. Border Mapping: The Border_Map function maps input values to specific ranges, allowing for control signal adjustments based on joystick and IMU readings.

12. Button Handling: It handles button presses for enabling/disabling IMU control and adjusting trim values. It includes debouncing to prevent multiple button presses.

13. Main Loop: The main loop continuously reads the joystick, updates trim values, handles button presses, calculates and sends control signals, manages the OLED display, and enters s 
    leep mode when inactive.

14. Display Control: The program controls the OLED display to show various information, including trim values, auxiliary channel values, and system status.

15. Initialization: Setup initializes various components, including the OLED display, radio module, GPIO pins, and the MPU6050 sensor.

16. ResetData: A function to reset control signals to default values if the connection is lost.

17. Updating Arduino Pro Mini Firmware Using ESP01 with OTA ( Comming Soon! )

# Receiver 3D IMAGE 

![Screenshot 2023-09-18 134145](https://github.com/HyperArx/Arduino-Transmitter-Receiver/assets/86643678/ea99f1ee-3162-425a-bd24-1693b3a2b001)
![Screenshot 2023-09-18 134218](https://github.com/HyperArx/Arduino-Transmitter-Receiver/assets/86643678/d91e2560-1855-461f-8238-2a0aa3840254)

![Screenshot 2023-09-18 133849](https://github.com/HyperArx/Arduino-Transmitter-Receiver/assets/86643678/028317d7-e2c3-4d8a-98c9-f3b1d13e1644)
![Screenshot 2023-09-18 133917](https://github.com/HyperArx/Arduino-Transmitter-Receiver/assets/86643678/2b8a5546-1c2a-4c96-98cb-bc6da7ae9cde)

# Transmitter 3D IMAGE 

![Screenshot 2023-09-18 133608](https://github.com/HyperArx/Arduino-Transmitter-Receiver/assets/86643678/1777a63e-7b49-465a-a2b8-a1fabc9be848)
![Screenshot 2023-09-18 133648](https://github.com/HyperArx/Arduino-Transmitter-Receiver/assets/86643678/bd1b8d34-d01f-4064-a3aa-88f128213253)

![Screenshot 2023-09-18 133715](https://github.com/HyperArx/Arduino-Transmitter-Receiver/assets/86643678/3c0b8628-505d-4a80-8b44-208bf762cb35)
![Screenshot 2023-09-18 133747](https://github.com/HyperArx/Arduino-Transmitter-Receiver/assets/86643678/3d6dc478-20d9-4829-97c0-83411b2b187b)

# Radio Controller For R.C Car

Download the below code if you are using Arduino Nano. For other Arduino Modules, you will have to change teh pin numbers according to your use.

[Remote.zip](https://github.com/HyperArx/Arduino-Transmitter-Receiver/files/7878967/Remote.zip) 

# Check NRF24L01

[nRF24L01_InspectionCode_-_elekkrypt.zip](https://github.com/HyperArx/Arduino-Transmitter-Receiver/files/7919641/nRF24L01_InspectionCode_-_elekkrypt.zip)




