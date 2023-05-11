#include<Wire.h> 
#include<Servo.h>

Servo right_pin; 
Servo left_pin ; 

  

int16_t accX, accY,accZ, gyroX, gyroY,filtacc0, filtacc,filtacc2,filtacc1 ;
float acc_angle[2];
float gyro_angle[2];
float tot_angle[2];
float elapsedtime, time, timeprev;
float conv = 180 / 3.141592654;

float pid, pwmleft, pwmright, error, previous_error;
float p=0;
float i=0;
float d=0;
/////////////////PID CONSTANTS/////////////////
double Cp=3.2;//3.55
double Ci=0.003;//0.003
double Cd=1.8;//2.05
///////////////////////////////////////////////
float val ;
float desiredangle = 0;                         

void setup(){
 
  Wire.begin();
  Wire.beginTransmission(0x68) ;
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  right_pin.attach(3); //attatch the right motor to pin 3
  left_pin.attach(5);  //attatch the left motor to pin 
   
 
  right_pin.writeMicroseconds(1000);
  left_pin.writeMicroseconds(1000);
 
 Serial.begin(9600);
  time = millis() ;

 
}


void loop(){
  timeprev = time ;
  time = millis() ;
  elapsedtime = (time - timeprev) / 1000 ; 
   
   
  ////////////////////////////////// accelerometer //////////////////////////////////
  Wire.beginTransmission(0x68) ;
  Wire.write(0x3B) ;
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true); 
  accX=Wire.read()<<8|Wire.read() ;
  accY=Wire.read()<<8|Wire.read() ;
  accZ=Wire.read()<<8|Wire.read();

  ////accelerometer X///////////
  acc_angle[0] = atan((accY/16384.0)/sqrt(pow((accX/16384.0),2) + pow((accZ/16384.0),2)))*conv ;
  ////accelerometer Y///////////
  acc_angle[1] = atan(-1*(accX/16384.0)/sqrt(pow((accY/16384.0),2) + pow((accZ/16384.0),2)))*conv;
  ///////////////accelerometer filter////////////////////////////////////////  
  filtacc0 = 0.9*filtacc + 0.1*acc_angle[0]  ;
  filtacc2 = 0.9*filtacc1 + 0.1* acc_angle[1] ;
  ////////////////////////////////// gyro //////////////////////////////////

 
  Wire.beginTransmission(0x68) ;
  Wire.write(0x43);
  Wire.endTransmission(false) ;
  Wire.requestFrom(0x68,4,true);
  gyroX=Wire.read()<<8|Wire.read();
  gyroY=Wire.read()<<8|Wire.read();
  ////accelerometer X///////////
  gyro_angle[0]=gyroX/131.0;
  ////accelerometer Y///////////
  gyro_angle[1]=gyroY/131.0;

  /////////////////////////////// complementery filter //////////////////////
  /*---X axis angle---*/
  tot_angle[0]=0.96*(tot_angle[0] + gyro_angle[0]*elapsedtime) + 0.04*filtacc0 ;
  /*---Y axis angle---*/
  tot_angle[1]=0.96*(tot_angle[1] + gyro_angle[1]*elapsedtime) + 0.04* filtacc2 ;

 //////////////////////////// PID //////////////////////////////////////////

 error =   tot_angle[0] - desiredangle ;

 p = Cp * error ;
 d = Cd * ((error - previous_error)/ elapsedtime) ; 
 if (-3 < error < 3 )
 {
  i = i + ( Ci * error) ;
 }

 pid = p + i + d ;

 if ( pid > 1000)
 {
  pid = 1000 ;
 }
 if (pid < -1000)
 { 
  pid = -1000 ;
 }

 
 

  /////////////////////////////////brushless control////////////////////////////
  
  pwmright = val + pid; 
  pwmleft = val - pid;
  
  if ( pwmright < 1000 )
  {
    pwmright = 1000 ; 
  } 
  if (pwmright > 2000)
  {
    pwmright = 2000 ;
  }
  if (pwmleft < 1000)
  {
    pwmleft = 1000 ;
  }
  if (pwmleft > 2000)
  { 
    pwmleft = 2000 ;
  }
   val = analogRead(A0);
  val = map(val, 0, 1023, 1000, 2000);
 
  right_pin.writeMicroseconds(pwmright);
  left_pin.writeMicroseconds(pwmleft);

  /* Serial.print(tot_angle[0]);
    Serial.print(", " ) ;
    Serial.print(pwmright) ;
    Serial.print(", ") ;
    Serial.print(pwmleft); 
    Serial.print(", ") ;
    Serial.println(val); */

 

  filtacc = filtacc0 ;
 filtacc1 =  filtacc2;
 previous_error = error ;
}
