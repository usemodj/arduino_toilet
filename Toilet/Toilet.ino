/*

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/

// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the left 
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

#include <Wire.h>
#include <DistanceGP2Y0A21YK.h>
#include <Servo.h>

// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1

//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 1   //Will print the Euler angles Roll, Pitch and Yaw

#define STATUS_LED 13 
#define UV_LED 12
#define UV_LED_TIME 60000  //5 minute(5*60*1000 msec)
#define OFF_DEG 7  //LED OFF Pitch(y) Degree
#define OFF_DIST 30  //LED Off distance cm
#define DIST_INPUT A0 //distance input pin A0
#define SERVO  9 //Servo pin 9
#define SERVO_OPEN 100  //Servo open degree
#define CLOSE_COUNT 500 // Distance sensor close counter
#define OPEN_COUNT 50  // Distance sensor open counter

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
long led_timer=0; //uv led timer
boolean led_active = true;
boolean servo_open = false;
int close_counter = 0;
int open_counter = 0;

DistanceGP2Y0A21YK Dist;
int distance;
Servo myservo; //create servo object to control a servo

int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors

int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z;
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

unsigned int counter=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};
 
void setup()
{ 
  Serial.begin(115200);
  pinMode (STATUS_LED,OUTPUT);  // Status LED
  pinMode (UV_LED,OUTPUT);  // UV LED
  
  digitalWrite(UV_LED,LOW);
  Dist.begin(DIST_INPUT);
  myservo.attach(SERVO); //Attaches the servo on pin 9 to the servo object
  myservo.write(0); // servo close
  delay(20);
  servo_open = false;
  close_counter = 0;
  led_active = true;
  servo_open = false;
  
  I2C_Init();
  Serial.println("NodeSoft Toilet");

  digitalWrite(STATUS_LED,LOW);
  delay(1500);
 
  Accel_Init();
  Compass_Init();
  Gyro_Init();
  
  delay(20);
  
  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }
    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;
    
  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
  
  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
    Serial.println(AN_OFFSET[y]);
  
  delay(2000);
  digitalWrite(STATUS_LED,HIGH);
    
  led_timer = millis();
  timer=millis();
  delay(20);
  counter=0;
}

void loop() //Main Loop
{
  if((millis()-timer)>=20)  // Main loop runs at 50Hz
  {
    counter++;
    timer_old = timer;
    timer=millis();
    if (timer>timer_old)
      G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    
    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer
    
    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
      {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading  
      }
    
    // Calculations...
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
    // ***
   
    printdata();
    Serial.println(digitalRead(UV_LED));
    distance = Dist.getDistanceCentimeter();
    Serial.print("Distance in centimers: ");
    Serial.println(distance);  
    if(!servo_open && distance < OFF_DIST){
      digitalWrite(UV_LED, LOW);
      close_counter = 0;
      open_counter++;
      if(open_counter > OPEN_COUNT){
        open_counter = 0;
        openToilet();
      }
      
    } else if(servo_open && distance > OFF_DIST + 20){
      open_counter = 0;
      close_counter++;
      if(close_counter > CLOSE_COUNT) {
        close_counter = 0;
        closeToilet();  
      }     
    } 
    
    if(abs(ToDeg(pitch)) < OFF_DEG){ //y-axis tilt
      
      if(!servo_open && led_active && digitalRead(UV_LED) == LOW){
        digitalWrite(UV_LED, HIGH);
        led_timer = millis();
      }
      if(millis() - led_timer > UV_LED_TIME){
        digitalWrite(UV_LED, LOW);
        led_active = false;
      }
      
    } else {
      digitalWrite(UV_LED, LOW);
      led_active = true;
    }
    
  }
   
}

void openToilet(){
  myservo.write(SERVO_OPEN/4);
  delay(100);
  myservo.write(SERVO_OPEN/2);
  delay(100);
  myservo.write(SERVO_OPEN*3/4);
  delay(100);
  myservo.write(SERVO_OPEN); //Servo open
  delay(100);
  servo_open = true;
}

void closeToilet(){
  myservo.write(SERVO_OPEN*3/4);
  delay(100);
  myservo.write(SERVO_OPEN/2);
  delay(100);
  myservo.write(SERVO_OPEN/4);
  delay(100);
  myservo.write(0); //Servo close
  delay(100);
  servo_open = false;  
}
