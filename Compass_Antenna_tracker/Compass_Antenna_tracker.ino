#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;
//#include <utility/imumaths.h>

#define EN 3
#define steps 9
#define dir 8
#define limit 2
int TotalstepCount = 0;
int endToEndStepCount = 0;
int adjestedStepCount = 0;
int directionalStepCount = 0;
int currentHeading = 0;     //
float angleToSteps = 0;
const int NorthToCalEndOffset = 45;
float stepsToAngle =0;

int once = 0;
int lastHeading = 0;
int setHeading =0;
bool northSet = false;
float P = 0.9;
float D = 0.01;
bool compassReady =false;
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address

int lastError = 0;
 float x;
    float y; 
    float z; 
    
    float roll;
    float pitch;
    float yaw;

void setup() {
  // put your setup code here, to run once:
pinMode(EN,OUTPUT);
pinMode(steps,OUTPUT);
pinMode(dir,OUTPUT);
pinMode (limit,INPUT);

digitalWrite(EN,LOW);
digitalWrite(dir,HIGH);
Serial.begin(115200);
  Wire.begin();
  myIMU.begin();
  Wire.setClock(100000); //Increase I2C data rate to 400kHz

  myIMU.enableMagnetometer(5); //Send data update every 50ms
  myIMU.enableRotationVector(5); //Send data update every 50ms
  
  Serial.println(F("Compass enabled"));
  
calibrate();

angleToSteps = endToEndStepCount / 360.0;
stepsToAngle = 360.0/endToEndStepCount;

Serial.print ("Enter heading :");
Serial.setTimeout(10);
}

void loop() {
  //while (!Serial.available());
    int HeadingReceived = Serial.parseInt();
    if (HeadingReceived == 1000){
      calibrate();
    }
    if ((HeadingReceived >=0) & (HeadingReceived < 361)){
    if (HeadingReceived != 0){
      setHeading = HeadingReceived;
    }
    if (setHeading != lastHeading){
        moveToHeading (setHeading);
       Serial.println ("Setting heading :");
      }
    }
    else{
      Serial.println("invalid input");
    }
    lastHeading = setHeading;
  //Serial.println(setHeading);
 // deactivatemotor();

  //............................................//
   
    
  if (myIMU.dataAvailable() == true)
  {
    x = myIMU.getMagX();
    y = myIMU.getMagY();
    z = myIMU.getMagZ();
    
    roll = (myIMU.getRoll()) ; // Convert roll to degrees
    pitch = -1 * (myIMU.getPitch()); // Convert pitch to degrees
    yaw = (myIMU.getYaw()) ; // Convert yaw / heading to degrees

  }
  
  int heading = atan2(y,x) * 180 / 3.14159 ;
  
  float Mpitch = -roll;
  float Mroll = pitch;
  float Xhorizontal = (x * cos(Mpitch)) + (y* sin (Mroll)*sin(Mpitch)) - (z*cos(Mroll) * sin(Mpitch));
  float Yhorizontal = y *cos(Mroll) + z*sin(Mroll);


  
  int compensatedHeading = atan2(Yhorizontal,Xhorizontal) * 180 / 3.14159 ;
if(compensatedHeading != 0){
      compassReady = true;
    }
  if (heading < 0){
    heading = 360.0 + heading;
  }
  if (compensatedHeading < 0){
    compensatedHeading = 360.0 + compensatedHeading;
  }
Serial.println(compensatedHeading);
  if ((compensatedHeading == 0) & (compassReady)){
    northSet = true;
  }

  if (northSet){
    int Error = setHeading - compensatedHeading;
    if (abs(Error - lastError) > 3){
      moveToHeading (Error);
      lastError = Error;
    }
    else{
      deactivatemotor() ;      //remove this when deploying
    }
    
  }
  //........................................................
}

void moveToHeading (int heading){
  activatemotor();
  int CWerror = heading-currentHeading; 
  int CCWerror = 360 - (heading-currentHeading);
  int s =1;
  currentHeading = heading;
  
  if (CWerror < 0){
    if((-1*CWerror)>(720-CCWerror)){
    int stepsToMove = (720-CCWerror) * angleToSteps;
    if (stepsToMove<40){
      s=2;
    }
    moveStepsCW(stepsToMove, s);
    }
    else{
    int stepsToMove = -1*CWerror * angleToSteps;
    if (stepsToMove<40){
      s=2;
    }
    moveStepsCCW(stepsToMove, s);
    //Serial.print (-1*CWerror);
    }
  }
  
  if ((CWerror > 0) & (CCWerror >0 )){
  if (CWerror < CCWerror){
    int stepsToMove = CWerror * angleToSteps;
    if (stepsToMove<40){
      s=2;
    }
    moveStepsCW(stepsToMove, s);
  }
  else{
    int stepsToMove = CCWerror * angleToSteps;
    if (stepsToMove<40){
      s=2;
    }
    moveStepsCCW(stepsToMove, s);
  }
  }
}

void calibrate(){
  currentHeading=0;
  setHeading =0;
  activatemotor();
  if (onLimit()){
    while(onLimit()){
      stepCCW(4);
    }
    for(int i =0;i<100;i++){
      stepCCW(4);
    }
  }
  while (!onLimit()){
    stepCW(1);
  }
  delay(400);
  TotalstepCount = 0;
  while (onLimit()){
    stepCW(1);
  }
  while (!onLimit()){
    stepCW(1);
  }
  endToEndStepCount = TotalstepCount;
  moveStepsCW(NorthToCalEndOffset,4);
  Serial.print ("end to end :");
  Serial.println (TotalstepCount);
}

void stepCW (int Speed){
  digitalWrite(dir,LOW);
  digitalWrite(steps,HIGH);
  delay(Speed);
  digitalWrite(steps,LOW);
  delay(Speed);
  TotalstepCount++;
  directionalStepCount++;
}
void stepCCW (int Speed){
  digitalWrite(dir,HIGH);
  digitalWrite(steps,HIGH);
  delay(Speed);
  digitalWrite(steps,LOW);
  delay(Speed);
  TotalstepCount++;
  directionalStepCount--;
}

void activatemotor(){
  digitalWrite(EN,LOW);
}
void deactivatemotor(){
  digitalWrite(EN,HIGH);
}

bool onLimit(){
  if (digitalRead(limit)){
    return true;
  }
  else{
    return false;
  }
}

void moveStepsCW(int Steps, int Speed){
  for (int i = 0;i < Steps+1;i++){
    stepCW(Speed);
  }
}
void moveStepsCCW(int Steps, int Speed){
  for (int i = 0;i < Steps+1;i++){
    stepCCW(Speed);
  }
}
