// IOT RalliDroid  
// Copyright Jukka Raivio and Jussi Salonen 2016
// MIT License
// 
// Example code is based on Arduino Mega2560 
//
// This arduino demo setup is connected to esp8266 Wifi via serial. 
// The http services are managed by esp-link firmware in ESP. The esp unit is working as Wifi-Serial bridge.
// No code changes are required for the esp-link
// Github repository for esp-link https://github.com/jeelabs/esp-link

// Debugging mode, output visible via e.g arduino IDE serial monitor (true = on, false = off)
boolean debugging = true;

// Temperature & Humidity sensor DHT11 lib & setup
// Adafruit version, https://github.com/adafruit/DHT-sensor-library
// Json description: https://github.com/jraivio/IoT-Ralli-Vempain/wiki
#include "DHT.h"
#define DHTPIN  2       // 2 digital pin 
#define DHTTYPE DHT11   
DHT dht(DHTPIN, DHTTYPE);

// I2C library
#include <Wire.h>
// RTC library by adafruit https://github.com/adafruit/RTClib
#include "RTClib.h"
RTC_DS1307 RTC;
String TimeStr ="";

// Multitasking functions by Arduino millis
// Initialization to enable multithreading features by using millis() functions  
// Sensor update interval
unsigned long previousMillis = 0;       // will store last time sensor updated
// the follow variables is a long because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
const long interval = 10000;            // Sensor data sending interval

// Json parser lib Copyright Benoit Blanchon 2014-2016, MIT License
// https://github.com/bblanchon/ArduinoJson
// Json description: https://github.com/jraivio/IoT-Ralli-Vempain/wiki
#include <ArduinoJson.h>
const char* command;
// Serial data handling, global variables
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
char inChar; // a char to handle incoming data

// RFID tag reader (NFC) MFRC522
/* ------------------------------------
  * RFID Card reader MFRC522 library by Miguel Balboa https://github.com/miguelbalboa/rfid
  * Signal      MFRC522      Mega 2560
  *             Pin          Pin       ´
  * -------------------------------------
  * RST/Reset   RST          49         
  * SPI SS      SDA(SS)      53       
  * SPI MOSI    MOSI         51        
  * SPI MISO    MISO         50        
  * SPI SCK     SCK          52        
*/
#include <SPI.h> // Arduino default SPI library
#include <MFRC522.h> 
#define RST_PIN   49     // Configurable, see typical pin layout above (valk)
#define SS_PIN    53    // Configurable, see typical pin layout above (vihr)
MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key; 
// Init array that will store new NUID 
byte nuidPICC[3];

/* Gyroscope + accelerometer MPU-6050 library for Arduino by Jeff Rowberg
 * Compass (magneto) HMC5883L.h library for Arduino by Jeff Rowberg
 * https://github.com/jrowberg/i2cdevlib
 */
#include <I2Cdev.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <MPU6050.h> //  Gyroscope + accelerometer
MPU6050 accelgyro(0x69); // Due to this AD0 pin must be pulled high otherwise 0x68 as default

#include <HMC5883L.h> // Magneto
HMC5883L magneto;
// Global variables to store the values
int16_t ax, ay, az; // ACC
int16_t gx, gy, gz; // Gyro
int16_t mx, my, mz; // Megneto

// Ultrasonic HC-SR04 library for Arduino by J.Rodrigo https://github.com/JRodrigoTech/Ultrasonic-HC-SR04
// Json description: https://github.com/jraivio/IoT-Ralli-Vempain/wiki
// Max distance 51 cm
#include <Ultrasonic.h>
#define TRIG_PIN 10     // Configurable Arduino pins for HC-SR04 Trig PIN
#define ECHO_PIN 9  // Configurable Arduino pins for HC-SR04 Echo pins
Ultrasonic ultrasonic(TRIG_PIN,ECHO_PIN); 

// Edge sensor pinouts
#define right_edge 23 // Right sensor
#define left_edge 22 // Left sensor

// Motor driver Setup
// math library for motor control
#include <math.h>
// Pins L298N -> Mega board
int enA = 8;
int in1 = 4;
int in2 = 5;
int in3 = 6;
int in4 = 7;
int enB = 3;
boolean motor_active = false;
// for motor delay
unsigned long mStartMillis = 0;      // will store last time motor delay update
int rightpower; // right motor power (0-100%) 
int leftpower; // left motor power (0-100%)
int mspeed; // motor speed
int mdelay; // motor delay
// millis for sensor reading interval while moving, delay 500 ms
boolean sensor_moving = false;
int sensor_moving_reports = 500;
unsigned long sensor_moving_previousMillis = 0;      // will store last time light delay update
// light blinking 
boolean pin13_blinking = false;
boolean pin12_blinking = false;
boolean pin11_blinking = false;
int pin13_delay;
int pin12_delay;
int pin11_delay;
unsigned long pin13_previousMillis = 0;      // will store last time light delay update
unsigned long pin12_previousMillis = 0;      // will store last time light delay update
unsigned long pin11_previousMillis = 0;      // will store last time light delay update

void JsonReportSensorDHT() {
  // DHT functions
  // Create Json and publish
  String jsonDHT = String("{\"sensor\":\"temp_hum\",\"time\":\"" + TimeStr + "\",\"data\":[" + dht.readTemperature() + "," + dht.readHumidity() + "]}");
  if (debugging == true) { Serial.println(jsonDHT);}
  Serial1.println(jsonDHT);
  return;
}

void JsonReportSensorDistance(){
  // Create Json and publish
  String jsonDist = String("{\"sensor\":\"distance\",\"time\":\"" + TimeStr + "\",\"data\":[" + ultrasonic.Ranging(CM) + "]}");
  if (debugging == true) { Serial.println(jsonDist);}
  Serial1.println(jsonDist);
  return;
}  

void JsonReportSensorAccAndGyro(){
  // read raw accelerator/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // Create Json and publish
  String jsonAccGyro = String("{\"sensor\":\"acc_gyro\",\"time\":\"" + TimeStr + "\",\"data\":[" + ax + "," + ay + "," + az +"," + gx +"," + gy + "," + gz + "]}");
  if (debugging == true) { Serial.println(jsonAccGyro);}
  Serial1.println(jsonAccGyro);
  return;
}

void JsonReportSensorMagneto(){
  // read raw magneto measurements from device
  magneto.getHeading(&mx, &my, &mz);
  // Create Json and publish
  String jsonMagneto = String("{\"sensor\":\"magneto\",\"time\":\"" + TimeStr + "\",\"data\":[" + mx + "," + my + "," + mz + "]}");
  if (debugging == true) { Serial.println(jsonMagneto);}
  Serial1.println(jsonMagneto);
  return;
}

void JsonReportSensorEdge() {
  // Create Json and publish
  String jsonEdge = String("{\"sensor\":\"edge\",\"time\":\"" + TimeStr + "\",\"data\":[" + digitalRead(left_edge) + "," + digitalRead(right_edge) + "]}");
  Serial.println(jsonEdge);
  if (debugging == true) { Serial.println(jsonEdge);}
  Serial1.println(jsonEdge);
  return;
}  
void JsonReportSensorRFID() {
  if ( ! rfid.PICC_ReadCardSerial()) return;
  // Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  // Serial.println(rfid.PICC_GetTypeName(piccType));
  // Check is the PICC of Classic MIFARE type
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&  piccType != MFRC522::PICC_TYPE_MIFARE_1K && piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    Serial.println(F("Your tag is not of type MIFARE Classic."));
    return;
  }
  if (rfid.uid.uidByte[0] != nuidPICC[0] || rfid.uid.uidByte[1] != nuidPICC[1] || 
    rfid.uid.uidByte[2] != nuidPICC[2] || rfid.uid.uidByte[3] != nuidPICC[3] ) {
    // Serial.println(F("A new card has been detected."));
    // Store NUID into nuidPICC array
    for (byte i = 0; i < 4; i++) { nuidPICC[i] = rfid.uid.uidByte[i];
    }   
    byte *buffer = rfid.uid.uidByte; 
    byte bufferSize = rfid.uid.size;
    String NUID = "";
    for (byte i = 0; i < bufferSize; i++) {
      NUID = String(NUID + String(buffer[i], DEC)); 
    }
    // Create Json and publish
    String jsonRfid = String("{\"sensor\":\"rfid\",\"data\":[" + NUID + "]}");
    if (debugging == true) { Serial.println(jsonRfid);}
    Serial1.println(jsonRfid);
    return;
  }
  else Serial.println(F("Card read previously."));
  rfid.PICC_HaltA(); // Halt PICC
  rfid.PCD_StopCrypto1(); // Stop encryption on PCD
  return;  
}

void driveMotors()
{
     /* This function will run the motors across the range of possible speeds.
     * Note that maximum speed is determined by the motor itself and the operating voltage. 
     * the PWM values sent by analogWrite() are fractions of the maximum speed possible by your hardware.
     * As an input value the relative definition of the used power range 0-100% which is converted to the corresponding PWM range 75-255. 
     * In theory the PWM range is 0-255. However in practice with this setup the operating range will be around 75-255 PWM. 
     * Formula: "PWM value to Motor driver" = "Min PWM" + (("Max PWM" - "Min PWM")/100))* Power%    
     * Driving direction can be defined by +/- values. Value (+) -> forward and Value(-) -> backward
     * Note that PWM operating range will be depending on several factors like capacity of batteries, overall system load, etc.
     * And finally each of the droid setup is different. Each of the devices are unique and requires case-by-case fine tunings.
     */
    boolean in1ena = (rightpower < 0)?false:true;
    boolean in2ena = (rightpower < 0)?true:false;
    boolean in3ena = (leftpower < 0)?false:true;
    boolean in4ena = (leftpower < 0)?true:false;
 
    int16_t rpower = (abs(rightpower*1.80))+75;  
    int16_t lpower = (abs(leftpower*1.80))+75; 

    if (rpower < 256 && lpower < 256 ) {
      digitalWrite(in1, in1ena); digitalWrite(in2, in2ena); digitalWrite(in3, in3ena); digitalWrite(in4, in4ena);
      analogWrite(enA, lpower); analogWrite(enB, rpower);
    }
}

void stopMotors() {
    // now turn off motors
    // clean up & return
    digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); digitalWrite(in3, HIGH); digitalWrite(in4, HIGH);
    delay(100);
    analogWrite(enA, 0); analogWrite(enB, 0);
    rightpower = 0;
    leftpower = 0;
    mdelay = 0;
    motor_active = false;
    // return;
}
/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent1() {
  while (Serial1.available()> 0) {
    inChar = (char)Serial1.read(); // get the new byte: 
    inputString += inChar; // add it to the inputString:
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
        stringComplete = true;
    }
  }
}
void HandleIncomingJson() {
    StaticJsonBuffer<512> jsonInBuffer;                 
    const char *JsonChar = inputString.c_str(); // 1 KB
    JsonObject& root = jsonInBuffer.parseObject(JsonChar);
    int pin;
    int value;
    int ldelay;
    
    // Verify Json 
    if (JsonChar!=NULL && !root.success()) {
      if (debugging == true) { 
       Serial.println("parseObject() failed: ");
       Serial.println(JsonChar); }
    }
    else {
        // Led pins 13-11
        command = root["command"];
        pin = root["data"][0];
        value = root["data"][1];
        ldelay = root["data"][2];
        // motor control
        rightpower = root["mdata"][0];
        leftpower = root["mdata"][1];
        mdelay = root["mdata"][2];
    if (command=="lights") {
    // Lights on, not blinking
      if (value == 1 && (ldelay == 0 || ldelay == NULL)){
        digitalWrite(pin, HIGH);
        if (pin == 13) { pin13_blinking = false; }
        if (pin == 12) { pin12_blinking = false; }   
        if (pin == 11) { pin11_blinking = false; }                
      }
      // blinking lights
        else if (value == 1 && (!ldelay == 0 || !ldelay == NULL)){
          if (pin == 13) { pin13_blinking = true; pin13_delay = ldelay;}
          if (pin == 12) { pin12_blinking = true; pin12_delay = ldelay;}   
          if (pin == 11) { pin11_blinking = true; pin11_delay = ldelay;}              
        }
        // Lights off
        else if (value == 0 ){
          digitalWrite(pin, LOW);
          if (pin == 13) { pin13_blinking = false; }
            if (pin == 12) { pin12_blinking = false; }   
            if (pin == 11) { pin11_blinking = false; }  
          }
        }
    if (command =="drive") {motor_active = true; mStartMillis = millis();}
    }
  // returning the default state of serialEvent1()
  stringComplete = false;
  // clean json incoming data buffers
  pin = NULL; value = NULL; ldelay = NULL;
  inputString="";
  inChar = NULL; JsonChar = NULL;
  return;      
}

void readTime() {

    DateTime now = RTC.now();
    String Year = String(now.year(), DEC);
    String Month = String(now.month(), DEC);
    String Day = String(now.day(), DEC);
    String Hour = String(now.hour(), DEC);
    String Minutes = String(now.minute(), DEC);
    String Seconds = String(now.second(), DEC);
    TimeStr = Year + "/" + Month + "/" + Day + ":" + Hour + ":" + Minutes + ":" + Seconds;
    // Debugging
    if (debugging == true) {Serial.println(TimeStr); }
    
}

void setup() {
    // initialize both serial ports:
    Serial.begin(115200); // Debugging
    delay(1000);
    Serial1.begin(115200); // Serial for esp8266
    delay(1000);
    Wire.begin(); // start I2C & RTC
    RTC.begin();
    if (! RTC.isrunning()) {
      if (debugging == true) {Serial.println("RTC is NOT running!");}
      // following line sets the RTC to the date & time this sketch was compiled
      //RTC.adjust(DateTime(__DATE__, __TIME__));
    }
    // initialize digital pin 13-11 as an output (for LEDs)
    pinMode(13, OUTPUT); // Main lights leds
    pinMode(12, OUTPUT); // Direction lights
    pinMode(11, OUTPUT);
    // Egde sensor input pins
    pinMode(right_edge,INPUT); // Right sensor
    pinMode(left_edge,INPUT); // Left sensor
    inputString.reserve(256); // reserve 256 bytes for the inputString:
    dht.begin(); // Init DHT
    magneto.initialize(); // magneto init
    accelgyro.initialize(); // acc & gyro init
    // RFID setup
    SPI.begin(); // Init SPI bus
    rfid.PCD_Init(); // Init MFRC522 
    for (byte i = 0; i < 6; i++) { key.keyByte[i] = 0xFF; } // RFID byte handling
}
void loop() {
    // update interval
    unsigned long currentMillis = millis();
    // DHT Sensor reporting
    if (currentMillis - previousMillis >= interval) {
      // save the last time of sensor reporting
      readTime();
      previousMillis = currentMillis;    
      JsonReportSensorDHT();
      if (motor_active == false) {
       JsonReportSensorDistance(); 
       JsonReportSensorEdge();
       JsonReportSensorAccAndGyro();
      } // slow down reporting frequency in static cases
    }
    if (motor_active == true) { // speed up reporting frequency in case of moving
      currentMillis = millis();
      if (currentMillis - mStartMillis >= sensor_moving_reports ) {
        readTime();
        JsonReportSensorDistance();
        JsonReportSensorEdge();
        JsonReportSensorAccAndGyro();
      }
    }
    // Look for new RFID cards
    if ( rfid.PICC_IsNewCardPresent()) { JsonReportSensorRFID(); }
    // Handling incoming Json from serial port 
    // JSon serial read, print the string when a newline arrives:
    serialEvent1(); if (stringComplete == true) { HandleIncomingJson();}
    
    // Light blinking
    if ( pin13_blinking == true){
      currentMillis = millis();
      if ( currentMillis - pin13_previousMillis >= pin13_delay ) {
          pin13_previousMillis = currentMillis; 
          if (digitalRead(13) == LOW) { digitalWrite(13, HIGH);
          } else { digitalWrite(13, LOW); }
        }
    }                       
    if ( pin12_blinking == true){
      currentMillis = millis();
      if (currentMillis - pin12_previousMillis >= pin12_delay) {
        pin12_previousMillis = currentMillis; 
        if (digitalRead(12) == LOW) { 
          digitalWrite(12, HIGH);
        } else { 
          digitalWrite(12, LOW); }
        }             
    }
    if ( pin11_blinking == true){
      currentMillis = millis();
      if ( currentMillis - pin11_previousMillis >= pin11_delay) {
        pin11_previousMillis = currentMillis; 
        if (digitalRead(11) == LOW) { 
          digitalWrite(11, HIGH);
        } else { 
          digitalWrite(11, LOW); }
        }             
    }     
    // Drive motors
    if (motor_active == true ) {
       currentMillis = millis();
       driveMotors();
       if (currentMillis - mStartMillis >= mdelay) {       
        stopMotors(); }           
    }  
 }
