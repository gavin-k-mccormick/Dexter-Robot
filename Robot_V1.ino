/*******************************************************************************************************
 * 
 *        Robot V1.0 - 
 *        
 *        Target Processor: ESP866Wifi
 *        Compiler: Arduino 1.8.1 + 
 *        Author: Gavin McCormick 
 *        Rev notes:
 *          1/28/2017 - created. webserver, i2c display initialization.
 *          3/1/2017 - Added Sd card to store webpage scripts and images
 *          
 *        Includes information:
 *        
 *        - Liquid Crystal library from https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads 
 *        GNU General Public License, version 3 (GPL-3.0)
 *        Pin Connections: 
 *          SCL = A5
 *          SDA = A4
 *          VCC = 5V
 *          GND = GND
 *          
 *          
/******************************************************************************************************/

/**** Include libraries *******************************************************/
#include <ESP8266WiFi.h>          // wifi module library
#include <Wire.h>                 // i2c include
#include <LCD.h>                  // lcd library
#include <LiquidCrystal_I2C.h>    // communcation via 12c fonctions
#include <Servo.h>                // pwm driven servo library
#include <SPI.h>                  // SPI include
#include <SD.h>                   // SD card interface library

/**** Pin Definitions - Easier in one place ***********************************/
const int ANALOG_PIN = A0;        // The only analog pin

// servos pin defs
const int SERVO_1_PIN = 2;        // PWM on GIO2
const int SERVO_2_PIN = 16;       // PWM on GIO16

// ultra sonic sensor pin defs
const int ULTRA_ECHO_PIN = 15;    // sound pulse output ctrl pin
const int ULTRA_TRIG_PIN = 0;     // feedback input pin

const int SD_CS_PIN = 0;          // SD card Chip Select on GIO9

/**** SD card declarations ****************************************************/
File scriptFile;


/**** Display declarations ****************************************************/
// declare a new instance of liquidcrystal via I2C bus
// 0x27 is the I2C bus address for an unmodified module
LiquidCrystal_I2C  lcd(0x27,2,1,0,4,5,6,7);


/**** Wifi declarations *******************************************************/
WiFiServer server(80);            // New instance of wifiserver 'server' on port 80
/** definitions **/
const char WiFiAPPSK[] = {
  "dextertherobot"                // WIFI PASSWORD 
}; 

const long CLIENT_TOUT = 20000;   // time before client goes idle
long clientToutTmr;               // timer for that.

char cssfile[] = "robot.css";
char imgFile[] = "dexter.JPG";
bool clientAttached = 0;          // flag for wifi client connection

/**** Servo declarations ******************************************************/
Servo servo1;                     // new servo lbr. obj. to control x axis servo 
Servo servo2;                     // ... y axis servo 

typedef struct {
  int angle = 0;                  // servo position in degrees
  int newAngle = 0;               // desired angle 
}servo_str;

servo_str servo_x;                // create a new var struct for x axis servo
servo_str servo_y;                // ... y axis servo

/** definitions **/
const char SERVO_Y_IDLE = 90;     // y axis angle for sleep mode
const char SERVO_X_IDLE = 90;     // x axis angle...
const char SERVO_Y_READY = 70;    // y axis angle for when object in initially sensed 

const char SERVO_X_MIN = 20;      // min boundary for x axis angle
const char SERVO_X_MAX = 160;     // max boundary...
const char SERVO_Y_MIN = 40;      // min boundary for y...
const char SERVO_Y_MAX = 95;      // max boundary...


/**** ultra sonic distance sensor declarations ********************************/
const long UPDATE_ULTRA = 5000;

typedef struct {
  int objGoneRange = 60;           // maximum range before robot doesn't care 
  int objSensedRange = 40;         // threshold before robot gets interested
  
  long distance = 0;               // distance measurement 
  long duration = 0;               // duration of sound wave flight used for meas.
  
  char scanCount = 0;              // directional scan indicator
  char scanDone = 0;               // are we done looking for object flag
  char scanDir = 0;                // sets x axis scan direction
  char initScan = 0;               // is this the first time through the scan loop flag
  char objSensed = 0;              // look, the robot sensed an object flag
   
  struct{
    int randomT = 0;
  }tmrs;
  
}ultra_str;
ultra_str ultra;

int ultraTmr;


/**** Core Global Variables ***************************************************/
int sleep = 0;                      // robot sleep flag


/******************************************************************************
 * Function: setup()
 *  Main setup routine. This runs before loop function only once.
 * 
 ******************************************************************************/
void setup() 
{
  clientToutTmr = CLIENT_TOUT;        // initialize timers
  ultraTmr = UPDATE_ULTRA;
 
  initHardware();                     // initializes cpu core and pins
  
  lcd.setBacklightPin(3,POSITIVE);
  lcd.setBacklight(HIGH);             // Turn on the LCD backlight 
  lcd.begin(20, 4);                   // initialize the lcd to 20x4 size
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("...core.......loaded");
  Serial.println("\r\n\r\n       DEXTER :: Test Robot V1.0\r\n");
  
  // initialize wifi module
  setupWiFi();                        
  lcd.setCursor(0,1);
  lcd.print("...wifi...........ON");
  Serial.println("...wifi...........ON\r");
  delay(1000);
  
  // connect to SD card. try to mount SD via SPI
  if (SD.begin(SD_CS_PIN)) {
    Serial.println(".");
    Serial.println("SD card Connected.");
    Serial.println("Webpage CSS script:");
    lcd.setCursor(0,2);
    lcd.print("...SD card Connected");

    // open the webpage css file. 
    scriptFile = SD.open(cssfile, FILE_READ);    
    if (scriptFile){                      // file found
      while (scriptFile.available()) {    // puke file to serial port
        Serial.write(scriptFile.read());
      }           
      scriptFile.close();                 // close the file
    }        
    else{                                 // file not found
      Serial.println(".");
      Serial.println("no file on SD card!");
      lcd.setCursor(0,2);
      lcd.print("...SD card...No File"); 
    }      
  }
  else {
    Serial.println(".");
    Serial.println("SD card initialization failed!");
    lcd.setCursor(0,2);
    lcd.print("...SD card....Failed");        
  }
  delay(1000);                            // visualization delay
  
  // begin webserver 
  server.begin();                     
  lcd.setCursor(0,3);
  Serial.println("");  
  Serial.println("");
  Serial.println("...Web server.....ON\r"); 
  lcd.print("...Web server.....ON");
  delay(1000);

  // attach servos
  servo1.attach(SERVO_1_PIN);           // attaches servo 1 to the GPIO pin 
  servo2.attach(SERVO_2_PIN);           // attaches servo 2 to the GPIO pin
  
  // shift up screen
  lcd.setCursor(0,0);

  lcd.print("...wifi...........ON");
  lcd.setCursor(0,1);
  lcd.print("...SD card Connected");
  lcd.setCursor(0,2);
  lcd.print("...Web server.....ON"); 
    
  // move servos to home position, alert user
  initServos(); 
  lcd.setCursor(0,3);
  lcd.print("...Servos.........ON");  
  delay(1000);

  // shift up screen
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("...SD card Connected");
  lcd.setCursor(0,1);
  lcd.print("...Web server.....ON");
  lcd.setCursor(0,2);
  lcd.print("...Servos.........ON");

  // update ultrasonic distance sensor.
  getUltraDist();
  lcd.setCursor(0,3);
  lcd.print("...Ultra sonic....ON");
  delay(1000);

  // shift up screen
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("...Web server.....ON");
  lcd.setCursor(0,1);
  lcd.print("...Servos.........ON");
  lcd.setCursor(0,2);
  lcd.print("...Ultra sonic....ON");
  
  // tell the user the system was booted successfully
  lcd.setCursor(0,3);
  lcd.print("...Bootup complete.");
  Serial.println(".");
  Serial.println("...Bootup complete!\r");
  Serial.println("\r\n\r\n\r\n");  
  delay(2000);
}

/******************************************************************************
 * Function: loop()
 *  Main program loop function. this runs continuously 
 * 
 ******************************************************************************/
void loop() 
{
  // checks for new servo position and starts move
  moveServo();

  /* **** ultraTmr ************************************************
   * update ultrasonic distance sensor when timer elapses
   * only enter if servo is not moving
   * ultra.distance is updated 
   */
  if (ultraTmr>0) ultraTmr--;
  if (!ultraTmr){
    getUltraDist();                             // update distance
   
    /* do stuff with distance measurement
     * we want to:
     *  find a person. ie, object sensed at certain distance
     *  stare at them... make them uncomfortable... then look for
     *  them once they move. 
     */     
    objScanner();  

    ultraTmr = UPDATE_ULTRA;                    // reset timer
  }  

  /* **** clientToutTmr ********************************************
   * default display when system is idle
   * statement entered every client_tout cnts
   */
  if (clientToutTmr>0) clientToutTmr--;
  if (clientToutTmr == 0){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Dexter V1.0 :Online");
    lcd.setCursor(3,1);
    lcd.print("Control Me at:");
    lcd.setCursor(4,2);
    lcd.print("192.168.4.1");
    lcd.setCursor(4,3);
    lcd.print("Distance: ");
    lcd.print(ultra.distance);
    Serial.println("OBJECT DISTANCE: " && ultra.distance && "cm \r\n");
       
    clientToutTmr = CLIENT_TOUT;
  }
  
  serverTasks();
  
  // The client will actually be disconnected 
  // when the function returns and 'client' object is destroyed
}

/******************************************************************************
 * Function: initHardware()
 *  Initializes serial port
 *  initializes GPIO 
 *  
 *  Returns void
 * 
 ******************************************************************************/
void initHardware()
{
  Serial.begin(115200);

  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_ECHO_PIN, INPUT);
  //pinMode(SD_CS_PIN, OUTPUT); don't need to set this since it shares ULTRA_TRIG_PIN 

}

/******************************************************************************
 * Function: setupWifi()
 *  opens new instance of the wifi module
 *  creates network ID with password WiFiAPPSK
 *  
 *  Returns void
 * 
 ******************************************************************************/
void setupWiFi()
{
  WiFi.mode(WIFI_AP);

  // Append the last two bytes of the MAC (HEX'd)":
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  String macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  String AP_NameString = "DEXTER V1.0." + macID;

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, 0, AP_NameString.length() + 1);

  for (int i=0; i<AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, WiFiAPPSK);
}

/******************************************************************************
 * Function: serverTasks()
 * updates webpage and runs client commands 
 *  
 *  Returns void
 * 
 ******************************************************************************/
void serverTasks()
{
  // Check if a client has connected
  WiFiClient client = server.available();
  if (!client) {
    clientAttached = 0;
    return;
  }  

  clientAttached = 1;
  
  // Read the first line of the request
  String req = client.readStringUntil('\r');
  client.flush();

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Request recieved: ");
  Serial.println("Request recieved: ");
  Serial.println(req);
  lcd.setCursor(5,2);  
 
 /*  // Match the request
  int val = -1; // We'll use 'val' to keep track of both the
                // request type (read/set) and value if set.
               
  if (req.indexOf("/led/0") != -1){
    val = 1; // Will write LED low
    servo_x.newAngle = 135;
    lcd.print("Turn Left");
  }
  else if (req.indexOf("/led/1") != -1){
    val = 0; // Will write LED high
    servo_x.newAngle = 45;
    lcd.print("Turn Right");
  }
  else if (req.indexOf("/read") != -1){
    val = -2; // Will print pin reads
  }
  // Otherwise request will be invalid. We'll say as much in HTML

  // Set GPIO5 according to the request
  //if (val >= 0) digitalWrite(LED_PIN, val);    

  client.flush();
  */
  if (req == "GET / HTTP/1.1"){ 
    // Prepare the response. Start with the common header:
    String respStr = "HTTP/1.1 200 OK\r\n";
    respStr += "Content-Type: text/html\r\n\r\n";
    respStr += "<!DOCTYPE HTML>\r\n<html>\r\n";
    respStr +=("<HEAD>\r\n");
    respStr +=("<meta name='apple-mobile-web-app-capable' content='yes' />\r\n");
    respStr +=("<meta name='apple-mobile-web-app-status-bar-style' content='black-translucent' />\r\n");
    respStr +=("<link rel='stylesheet' type='text/css' href='robot.css' />\r\n");
    respStr +=("<TITLE>Dexter - Robot v1.0 - </TITLE>\r\n");
    respStr +=("</HEAD>\r\n");
    respStr +=("<BODY>\r\n");
    //respStr +=("<img src='img.jpg' alt='Dexter' style='width:440px;height:210px;'/>\r\n");
    respStr +=("<H1>  DEXTER The Robot V1.0.0 :: Control Interface <br />\r\n");
    respStr +=("<br /> Target Platform: ESP8266wifi</H1>\r\n");
    respStr +=("<br />\r\n");  
    respStr +=("<H2>:: Robot Position ::");
    respStr +=("<br />X-Axis: \r\n");
    respStr += servo_x.angle;
    respStr += " degrees \r\n";
    respStr +=("<br />Y-Axis: \r\n");
    respStr += servo_y.angle;
    respStr += " degrees </H2>";
    respStr +=("<br />\r\n");  
    respStr +=("<br />\r\n");  
    respStr +=("<a href=\"/?button1on\"\">Rotate Up</a>\r\n");
    respStr +=("<a href=\"/?button1off\"\">Rotate Down</a><br />\r\n");   
    respStr +=("<br />\r\n");     
    respStr +=("<br />\r\n"); 
    respStr +=("<a href=\"/?button2on\"\">Scan Left</a>\r\n");
    respStr +=("<a href=\"/?button2off\"\">Scan Right</a><br />\r\n"); 
    respStr +=("<br />\r\n"); 
    respStr +=("<p>Created by Gavin McCormick</p>\r\n");  
    respStr +=("<br />\r\n"); 
    respStr +=("</BODY>\r\n");
    respStr +=("</HTML>\n");
  
    // Send the response to the client
    client.print(respStr);
  }
  else if (req == "GET /robot.css HTTP/1.1"){ 
    // open the webpage css file. 
    scriptFile = SD.open(cssfile, FILE_READ);
    // file found
    if (scriptFile){
      // puke file to serial port
      while (scriptFile.available()) {
        client.write(scriptFile.read());
      }
      
      // close the file
      scriptFile.close();  
    }   
  }
  else if (req == "GET /img.jpg HTTP/1.1"){
    // open the webpage css file. 
    scriptFile = SD.open(imgFile, FILE_READ);
    // file found
    if (scriptFile){
      // puke file to serial port
      while (scriptFile.available()) {
        client.write(scriptFile.read());
      }
      
      // close the file
      scriptFile.close();  
    }  
    else{
      Serial.println("connot locate image source for webpage request!\r\n"); 
    }
  }


  delay(1);

  //stopping client
  client.stop();

  // process HTML User input (: buttons)
  int pos;
  //controls the Arduino if you press the buttons
  if (req.indexOf("?button1on") >0){
    for(pos = SERVO_Y_MIN; pos < SERVO_Y_MAX; pos += 1)  // goes from 0 degrees to 180 degrees     
    {                                  // in steps of 1 degree 
      servo2.write(pos);               // tell servo to go to position in variable 'pos' 
      delay(10);                       // waits 15ms for the servo to reach the position 
    } 
  }
  if (req.indexOf("?button1off") >0){
    for(pos = SERVO_Y_MAX; pos>=SERVO_Y_MIN; pos-=1)     // goes from 180 degrees to 0 degrees 
    {                                
      servo2.write(pos);               // tell servo to go to position in variable 'pos' 
      delay(10);                       // waits 15ms for the servo to reach the position 
    } 
  }
  if (req.indexOf("?button2on") >0){
    for(pos = SERVO_X_MIN; pos < SERVO_X_MAX; pos += 1)  // goes from 0 degrees to 180 degrees 
    {                                  // in steps of 1 degree 
      servo1.write(pos);               // tell servo to go to position in variable 'pos' 
      delay(10);                       // waits 15ms for the servo to reach the position 
    } 
  }
  if (req.indexOf("?button2off") >0){
    for(pos = SERVO_X_MAX; pos>=SERVO_X_MIN; pos-=1)     // goes from 180 degrees to 0 degrees 
    {                                
      servo1.write(pos);               // tell servo to go to position in variable 'pos' 
      delay(10);                       // waits 15ms for the servo to reach the position 
    } 
  }            
  Serial.println("Client disonnected");  
}

/******************************************************************************
 * Function: initServos()
 *  Initializes servos by sweeping through their operating ranges and centering
 *  
 *  Returns void
 * 
 ******************************************************************************/
void initServos()
{
  // initialize servo position to current position
  servo_x.angle = servo1.read();
  servo_y.angle = servo2.read();

  // sweep the first servo from current position to 180
  // sweep the second from current position to 90 (center)
  for(servo_x.angle; servo_x.angle<SERVO_X_MAX; servo_x.angle++) 
  {                                   
    servo1.write(servo_x.angle);        // in steps of 1 degree
                                        // tell servo to go to position in variable 'pos' 
    if (servo_y.angle < SERVO_Y_MAX){   // at the same time, sweep servo 2 to 90degrees
      servo2.write(servo_y.angle);
      servo_y.angle++;
    }               
    delay(15);                          // waits 15ms for the servo to reach the position 
  } 
  // move the first servo to center
  // move the second from 90 to 40 
  for(servo_x.angle; servo_x.angle>SERVO_X_IDLE; servo_x.angle--) 
  {                                     // in steps of 1 degree 
    servo1.write(servo_x.angle);        // tell servo to go to position in variable 'pos' 
    
    if (servo_y.angle > SERVO_Y_MIN){   // at the same time, sweep servo #2 to 40degrees
      servo2.write(servo_y.angle);  
      servo_y.angle--;
    }              
    delay(15);                          // waits 15ms for the servo to reach position 
  }    
  servo_y.angle = SERVO_Y_IDLE;
  servo2.write(servo_y.angle);          // finally, move servo 2 to center
  
  servo_x.newAngle = servo_x.angle;     // initialize new angle to current angle
  servo_y.newAngle = servo_y.angle;
}

/******************************************************************************
 * Function: moveServo()
 *  checks if x or y-axis servos have a new position to move to, newAngle
 *  if so, function moves servo in 1 degree steps with 10ms delay between 
 *  to give a smooth motion
 *  
 *  Returns void
 * 
 ******************************************************************************/
void moveServo()
{
  // move x-axis servo to new position 
  if (servo_x.newAngle > servo_x.angle){
    for(servo_x.angle; servo_x.angle < servo_x.newAngle; servo_x.angle++) // in steps of 1 degree 
    {                                  
      servo1.write(servo_x.angle);         // tell servo to go to position in variable 'pos' 
      delay(10);                           // waits 10ms for the servo to reach the position 
    } 
  }
  else if (servo_x.newAngle < servo_x.angle){
    for(servo_x.angle; servo_x.angle > servo_x.newAngle; servo_x.angle--) // in steps of 1 degree 
    {                                  
      servo1.write(servo_x.angle);         // tell servo to go to position in variable 'pos' 
      delay(10);                           // waits 10ms for the servo to reach the position 
    } 
  }
  // move y-axis servo to new position   
  if (servo_y.newAngle > servo_y.angle){
    for(servo_y.angle; servo_y.angle < servo_y.newAngle; servo_y.angle++) // in steps of 1 degree 
    {                                  
      servo2.write(servo_y.angle);         // tell servo to go to position in variable 'pos' 
      delay(10);                           // waits 10ms for the servo to reach the position 
    } 
  }
  else if (servo_y.newAngle < servo_y.angle){
    for(servo_y.angle; servo_y.angle > servo_y.newAngle; servo_y.angle--) // in steps of 1 degree 
    {                                  
      servo2.write(servo_y.angle);         // tell servo to go to position in variable 'pos' 
      delay(10);                           // waits 10ms for the servo to reach the position 
    } 
  }
}

/******************************************************************************
 * Function: objScanner()
 *  looks at object within specified range then suedo-randomly looks around 
 *  once it leaves range. uses single ultrasonic sensor
 *  
 *  this is crude.
 *  
 *  Returns void
 * 
 ******************************************************************************/
void objScanner()
{ 
  int xAxisMoveInc = 5;
  
  // object found. stop scan and look at target. 
  if (ultra.distance < ultra.objSensedRange)
  {  
    sleep = 0;   
    if (!ultra.objSensed){                // is this the first time through?
      ultra.objSensed = 1;
      lcd.setBacklight(HIGH);             // turn on LCD backlight
      servo_y.newAngle = SERVO_Y_READY;   // 'look up' at person
    }     
    else {                                // object already sensed, tilt head
      randomYAxis();                      // to random yaxis point
      if (ultra.scanCount){
        ultra.scanCount = 0;              // reset direction change counter 
        ultra.scanDir != ultra.scanDir;   // flip scan direction
      }
    }    
  } 
  // object has moved from sensor range. start scan
  else if (ultra.distance >= ultra.objGoneRange && ultra.objSensed)
  {    
    if (!sleep) {
      randomYAxis();                      // get a new random Y axis for head tilt 
                                          // if robot isn't sleeping initialize new scan
      // print to terminal                                     
      Serial.println("SERVO X POSITION: ");   
      Serial.println(servo_x.angle);
      Serial.println("\r");
      Serial.println("SERVO YPOSITION: ");
      Serial.println(servo_y.angle);
      Serial.println("\r\n");
    }
                                               
    if (!ultra.initScan){                 
      ultra.initScan = 1;
      
      // get direction for the x-axis movement by getting random number and 
      // taking the remander when divided by 2 (modulus)
      ultra.scanDir = (random(1, 100) % 2);
    }
    // scan for object if we aren't done looking around
    else if (!ultra.scanDone){      
      // if dir flag set, move to the right... 
      if (ultra.scanDir){
        // ...but only if max angle hasnt been reached 
        if (servo_x.newAngle < SERVO_X_MAX-xAxisMoveInc) servo_x.newAngle+=xAxisMoveInc;
        // ...otherwise, check if this is the end of the scan, set flag if so.
        else if (ultra.scanCount)ultra.scanDone = 1;         
        else {                            // we reached the max, change scan direction
          ultra.scanCount=1;
          ultra.scanDir = 0;       
        }
      }       
      // if dir flag not set, move to the left...
      else { 
        // ... but only if min angle hasnt been reached 
        if (servo_x.newAngle > SERVO_X_MIN+xAxisMoveInc) servo_x.newAngle-=xAxisMoveInc;
        // ...otherwise, check if this is the end of the scan, set flag if so.
        else if (ultra.scanCount) ultra.scanDone = 1;
        else {                            // we reached the max, change scan direction
          ultra.scanCount=1;
          ultra.scanDir = 1;
        }
      }   
    } 
    else{                                 // scan is complete, no object found, lets 
      ultra.scanCount = 0;                // reset scan direction counter (...flag)
      ultra.scanDone = 0;                 // ... scan finished flag
      ultra.objSensed = 0;                // ... object sensed flag
      ultra.initScan = 0;                 // ... scan initialized flag
      servo_y.newAngle = SERVO_Y_IDLE;    // set y axis to idle point
      servo_x.newAngle = SERVO_X_IDLE;    // as well as the x axis
      lcd.setBacklight(LOW);              // turn off LCD backlight to save power
      sleep = 1;
    }            
  }   
}

/******************************************************************************
 * Function: ()
 *  
 *  
 *  Returns void
 * 
 ******************************************************************************/
void randomYAxis()
{
  /* get new random time before y-axis head movement
   * get new random y-axis value as well. */
  if (ultra.tmrs.randomT == 0){
    ultra.tmrs.randomT = random(3, 8);                      // new random time before movement      
    servo_y.newAngle = random(SERVO_Y_MIN, SERVO_Y_IDLE);   // new random Y-axis in degrees.
  }
  ultra.tmrs.randomT--;  
}

/******************************************************************************
 * Function: getUltraDist()
 *  sends a pulse through ultrasonic sensor
 *  calculates distance of object from sensor by determining duration of echo'd 
 *  pulse 
 *  
 *  Returns void
 * 
 ******************************************************************************/
void getUltraDist()
{
  /* The following trigPin/echoPin cycle is used to determine the
  distance of the nearest object by bouncing soundwaves off of it. */ 
  digitalWrite(ULTRA_TRIG_PIN, LOW); 
  delayMicroseconds(2); 
  
  digitalWrite(ULTRA_TRIG_PIN, HIGH);
  delayMicroseconds(10); 
   
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  ultra.duration = pulseIn(ULTRA_ECHO_PIN, HIGH);
   
  //Calculate the distance (in cm) based on the speed of sound.
  ultra.distance = ultra.duration/58.2;
 
}

