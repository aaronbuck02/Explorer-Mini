#include <SD.h>
#include <sd_defines.h>
#include <sd_diskio.h>

/*
Go Baby Go Adherence Sensor Master Code
Written by Ashlee B, Kyler M, Tara P 4/26/2023
Adapted by Margo B, Kaylee M, Anna Y 4/26/2024
*/


// General Libraries
#include <SPI.h>                                           // Communication with Serial Peripheral Interface (SPI) devices (ex. microcontroller and other circuits)
#include <Wire.h>                                          // I2C communication between chips
#include <SparkFun_RV8803.h>                               // Read and set time on the RTC
#include <Adafruit_Sensor.h>                               // Required library for all Adafruit Unified Sensor libraries
#include <Adafruit_LIS3DH.h>                               // Configures and communicates data from Adafruit LIS3DH accelerometer (uses Unifed Sensor Library)
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>  // Used to monitor external battery level
#include "FS.h"                // ESP 32 File System library
#include <SD.h>                // Potential Problem: the duplicate libraries
// #include "SPI.h"               // Part of the SPI library

//Pins
#define EVI 14     // Timestamp pin
#define R_LED 4    // Red LED pin
#define G_LED 13   // Green LED pin - changed from 16 to 13
#define B_LED 17   // Blue LED pin
#define BUTTON 25  // Button input pin

// Electronic component variables
RV8803 rtc;                               // Real Time Clock variable - uses the class RV8803
Adafruit_LIS3DH lis = Adafruit_LIS3DH();  // LIS3DH Accelerometer variable

SFE_MAX1704X battery;                     // External battery object, used to initiate low battery mode
double battery_percent;                   // Holds the current battery percentage
int low_battery_threshold = 20;           // Threshold for low battery indication

float accel;
int detect = 0;                 // Number of no-motion detections
bool det = false;               // Indicates a new motion was detected
float numReadings = 50.0;       // Used to average the accelerometer readings
const float gravity = 9.80665;  // Earth's gravity in m/s^2

// Hold the previous averaged readings for each axis on the accelerometer
float last_x;
float last_y;
float last_z;

// Hold the current averaged readings for each axis on the accelerometer
float x;
float y;
float z;

bool check = false;

// // Read state of the button press
// int buttonState = 0;

// Idle Mode: variables that measure the time since the last detected motion
unsigned long stop_motion;                           // Time when motion stopped
unsigned long current_time;                          // Current time

// DOESNT WORK
// lis3dh_dataRate_t current_rate = lis.getDataRate();  // Gets the current accelerometer data collection mode (for low power mode)

const uint8_t FLAG_REGISTER = 0x0E; // Flag register (datasheet)
const uint8_t VLF_MASK      = 0b00000011; // Bits 1 and 0: V2F + V1F
// The accelerationdata.txt file
File file;

void setup() {
  Serial.begin(115200);  // Set baud rate

  pinMode(R_LED, OUTPUT);
  pinMode(G_LED, OUTPUT);
  pinMode(B_LED, OUTPUT);
  pinMode(EVI, OUTPUT);  // Connecting to the Real-Time Clock
  Wire.begin();          // I2C addresses begin
 

  //RTC initialization
  if (rtc.begin() == false)  // If the RTC cannot be found or started
  {
    Serial.println(F("Device not found. Please check wiring. Freezing."));  // Prints to the serial monitor that the RTC cannot be found
    while (1);  // NEED INFORMATION: why are they using an infinite loop?
  }
  Serial.println(F("RTC online!"));  // Prints to the serial monitor that the RTC was found


  // DO NOT USE
  // if (rtc.setToCompilerTime() == false) {
  //   Serial.println(F("An error has occurred in setting the RTC to compiler time"));
  // }

  // // --- Read flag register ---
  // uint8_t flags = rtc.readRegister(FLAG_REGISTER);
  // Serial.print("Flag register: 0x");
  // Serial.println(flags, HEX);

  // if (flags & VLF_MASK) {
  //   Serial.println("RTC lost power or time invalid — setting new time...");
    

  //   // Clear the flag so this block won't run again
  //   rtc.writeRegister(FLAG_REGISTER, flags & ~VLF_MASK);

  //   Serial.println("RTC time set and saved.");
  // } else {
  //   Serial.println("RTC already keeping valid time.");
  // }


  // Comment out after running for the first time or reset to the current time
  rtc.setTime(0, 20, 15, 4, 23, 10, 2025); // seconds, minutes, hours, day of week(monday = 1), date, month, year
  

  rtc.setEVIEventCapture(RV8803_ENABLE);  //Enables the Timestamping function

  Serial.println(F("LIS3DH test!"));
  if (!lis.begin(0x19)) {                // If the LIS3DH accelerometer cannot be found or used, follow this condition
    Serial.println(F("Couldnt start"));  // Prints to the serial monitor that the accelerometer cannot be found
    while (1) yield();                   
  }
  Serial.println(F("LIS3DH found!"));  // The LIS3DH accelerometer is found

  // Set up lis and init readings
  lis.setRange(LIS3DH_RANGE_4_G);

  delay(10);   // Delays by 10 milliseconds
  lis.read();  // Takes a reading from the accelerometer



  // ADJUST THIS ORIENTATION!!!!
  x = -lis.z;  // Takes the x-axis reading, using a negated z-axis accelerometer reading
  y = -lis.x;   // Takes the y-axis reading, using a negated x-axis accelerometer reading
  z = -lis.y;  // Takes the z-axis reading, using a negated y-axis accelerometer reading




  // Check if SD is connected correctly
  if (!SD.begin()) {                         // If the SD card cannot be found, follow this condition
    Serial.println(F("Card Mount Failed"));  // Prints to the serial monitor that the SD card cannot be found
    return;                                  // Returns without further action
  }
  uint8_t cardType = SD.cardType();  // Saves the type of SD card to an unsigned 8-bit integer

  if (cardType == CARD_NONE) {                 // If there is no SD card, follow this condition
    Serial.println(F("No SD card attached"));  // Prints that there is no SD card attached
    return;                                    // Returns without further action
  }

  Serial.print("SD Card Type: ");      // Prints this statement to the serial monitor
  if (cardType == CARD_MMC) {          // If the card is a MultiMediaCard, follow this condition
    Serial.println(F("MMC"));          // Prints that the card is a MultiMediaCard (MMC) to the serial monitor
  } else if (cardType == CARD_SD) {    // If the card is a standard SD card, follow this condition
    Serial.println(F("SDSC"));         // Prints that the card is a standard SD card to the serial monitor
  } else if (cardType == CARD_SDHC) {  // If the card is a high capacity SD card, follow this condition
    Serial.println(F("SDHC"));         // Prints that the card is a high capacity SD card to the serial monitor
  } else {                             // If the card is of another type, follow this condition
    Serial.println(F("UNKNOWN"));      // Prints that the card is of an unknown type of the serial monitor
  }

  // Initialize the MAX17043 battery object
  if (battery.begin() == false) {
    Serial.println(F("Can't find MAX1704X Battery"));
  }

  // For low battery mode
  battery.quickStart();                         // Restarts the MAX1704X to createa. more accurate guess for the SOC
  battery.setThreshold(low_battery_threshold);  // An interrupte to alert when the battery reahes 20% and below

  //axis zeroing to eliminate false positives
  x = 0;
  y = 0;
  z = 0;

  // Procedure at device start-up
  String startTime = RTC();  //timestamp

  file = SD.open("/accelerationdata.txt", FILE_APPEND);  //open file.txt to write data
  if (!file) {                                           // If the file cannot be opened, follow this condition
    Serial.println(F("Could not open file(writing)."));  // Prints that the accelerometer.txt file cannot be opened
  }

  else {                                 // If the file can be opened, follow this condition
    file.println();                      // Adds a new line to the file
    String start_up = "Device start up at: " + startTime; // Encrypts the start-up time string

    Write(start_up);
    file.println(start_up);
    file.println();                      // Prints another new line to the bottom of the file
    file.close();                        // Closes the file
  }
}

void loop() {
  // Records whether the button has been pressed
  
  current_time = millis();

  // Gets the battery percent
  battery_percent = battery.getSOC();


  // LED Test
  int Red = analogRead(R_LED)/4;
  int Green = analogRead(G_LED)/4;



  /*
   * Condition 1: Idle Mode Check
   * Puts the accelerometer into low power mode if the cart has been idle for longer than 30 seconds
   * Conducts regular motion detection in low power mode
  */

  // if (current_time - stop_motion > 30000) {
  //   setColor(255, 30, 0);                            // Sets the color to "orange-ish"
  //   lis.setDataRate(LIS3DH_DATARATE_LOWPOWER_5KHZ);  // Sets the accelerometer to low-power mode
  //   motionDetection();                             // Resumes motion detection
  //   // Serial.print("Red:");
  //   // Serial.println(Red);
  //   // Serial.print("Green:");
  //   // Serial.println(Green);
  // }

  /*
   * Condition 2: Low Battery Check
   * Checks if the battery level is below 20% and displays a red LED
   * Conducts regular motion detection in full power mode
  */

  // Checks if battery is below 20 percent
  if (battery_percent < 20) {
    setColor(255, 0, 0);  // Sets the LED color to red
    motionDetection();    // Runs the motion detection

    // Serial.print("Red:");
    // Serial.println(Red);

    // Print the battery percentage
    Serial.print("Battery Percentage: ");
    Serial.print(battery_percent);
    Serial.println("%");
  }

  /*
   * Condition 3: Regular Motion Detection
   * Sets the LED color to green and conducts regular motion detection
  */

  else {
    setColor(0, 0, 255);                     // Sets the LED color to green
    lis.setDataRate(LIS3DH_DATARATE_10_HZ);  // Sets the accelerometer to normal data collection
    motionDetection();                       // Runs the motion detection
    // Serial.print("Green:");
    // Serial.println(Green);
  }
}

/*
 * void motionDetection
 * Reads accelerometer data and uses a threshold of change to determine if the child is moving
*/

// Accelerometer: Uses a delta scheme to detect movement
void motionDetection() {
  accelRead();        // Takes 50 accelerometer readings and averages them
  // Serial.println("Reading");
  

  if (check){
    Write("X: ");
    Write(String(x, 2));
    Write("Y: ");
    Write(String(y, 2));

  }
  
  // Threshold of change in each axis to be considered a detected movement
  float move_tholdX = 0.3;
  float move_tholdY = 0.3;
  float move_tholdZ = 0.3;// should be really high, don't care about z axis

  // If there is a significant change in any three axes, create a timestampe to record to SD card, and indicate the start of a new motion
  if ((abs(last_x - x) > move_tholdX || abs(last_y - y) > move_tholdY) || abs(last_z - z) > move_tholdZ  &&  det == false) {
    digitalWrite(EVI, HIGH);    // trigger EVI pin
    delay(20);                  // wait for a second
    digitalWrite(EVI, LOW);     // turn off EVI pin output
    Serial.print("Detected ");  //Debug purposes
    det = true;                 // Indicates a new motion was detected
    stop_motion = millis();
    sd_Start();  // Writes to the file when the motion was detected

    check = true;

    
  }

  // If there is no longer significant change in all three axes add to the detection counter
  else if (move_tholdX > abs(last_x - x) && move_tholdY > abs(last_y - y) && move_tholdZ > abs(last_z - z) && detect < 10 && det == true) {
    detect += 1;
  }

  // If after 300ms there is no more significant change, the time of stopped motion will be printed to the SD card
  else if (move_tholdX > abs(last_x - x) && move_tholdY > abs(last_y - y) && move_tholdZ > abs(last_z - z) && detect == 10 && det == true) {
    digitalWrite(EVI, HIGH);   // trigger EVI pin
    delay(20);                 // wait for a second
    digitalWrite(EVI, LOW);    // turn off EVI pin output
    Serial.print("Stopped ");  //Debug purposes
    sd_Stop();                 // Prints the time the motion stopped
    det = false;               // Resets the det boolean to false
    detect = 0;                // Resets the detect counter
    stop_motion = millis();    // Detects the time motion stops

    check = false;
  }

  else {         //if the data goes above the thresholds over reset detect counter
    detect = 0;  // Resets the detect counter
  }

  // Give time between readings
  delay(100);  // 100 milliseconds between readings
}

/*
 * String RTC
 * Creates a Real-Time Clock timestamp for recording the current time
*/

String RTC() {
  String time;  // The string used to save the timestamp data
  //RTC timestamp
  if (rtc.getInterruptFlag(FLAG_EVI)) {
    rtc.updateTime();  // Grabs an updated gime from the compiler
    rtc.clearInterruptFlag(FLAG_EVI);

    // Get the current date in mm/dd/yyyy format
    String currentTime = rtc.stringDateUSA();

    //Get the timestamp
    String timestamp = rtc.stringTimestamp();

    // Saves the date string and the time string
    time = currentTime + " " + timestamp;

    Serial.print(time);  // Prints the date and time to the serial monitor
    //end debug
  }
  return time;  // Returns the timestamp string
}

/*
 * void accelRead
 * Takes 50 accelerometer readings and averages them
*/

//read from the accelerometer function puts data through moving average filter
void accelRead() {
  sensors_event_t event;  // new sensor event

  // Sums the 50 samples for each axis
  float x_sum = 0;
  float y_sum = 0;
  float z_sum = 0;

  // Moving Average Filter- Take average of 50 samples for each axis
  for (int i = 0; i < numReadings; i++) {
    lis.getEvent(&event);  // Grabs the specific accelerometer data

    x_sum += (-event.acceleration.z);  // Adds this specific x-axis instance to the sum, using the negated z-axis accelerometer data
    y_sum += event.acceleration.y;     // Adds this specific y-axis instance to the sum
    z_sum += (-event.acceleration.x);  // Adds this specific z-axis instance to the sum, using the negated x-axis accelerometer data
    delay(2);                          // Delays for 2 milliseconds
  }

  // Assigns the previous readings to the 'last' versions of the specific axes
  last_x = x;
  last_y = y;
  last_z = z;

  // Divides the readings by 50 (because of the 50 readings), and assigns these values to their respective axis readings
  x = x_sum / numReadings;
  y = y_sum / numReadings;
  z = z_sum / numReadings;
}

void sd_Start() {  //prints to sd card starting

  String startTime = RTC();  // Grabs the initial timestamp using the RTC helper function
  Serial.println();          // Adds a new line in the serial monitor

  String start_ = "Start Time of Motion Detected At: " + startTime + "\n";  // String for the start time
  Write(start_);

  // TESTING
  Serial.println(start_);
}

/*
 * void sd_Stop
 * Prints the time of stopped motion to accelerationdata.txt
*/

void sd_Stop() {  //printing to the SD card

  String stopTime = RTC();  //timestamp
  Serial.println();         // Adds a new line

  // The acclerationdata.txt file could be read
  String stop_ = "Time of Stop Detected At: " + stopTime + "\n";
  Write(stop_); // Appends this to the bottom of the file when motion is stopped


  // TESTING
  Serial.println(stop_);
}



/*
 * void Write
 * Encrypts data and writes to the accelerationdata.txt file
*/

void Write(String data) {
  file = SD.open("/accelerationdata.txt", FILE_APPEND);  //open file.txt to write data
  if (!file) {                                           // If the accelerationdata.txt file couldn't be read, follow this condition
    Serial.println("Could not open file(writing).");     // Prints to the serial monitor that the file couldn't be opened
  }

  else {
    file.println(data);     // Prints data to the file
    file.close();  // Closes accelerationdata.txt file
  }
}
 

/*
* void setColor
* Sets the color of the RGB LED
*/

void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(R_LED, redValue);
  analogWrite(G_LED, greenValue);
  analogWrite(B_LED, blueValue);
}
