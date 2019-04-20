#include <Wire.h>
//#include <ESP8266WiFi.h>
//--- Accelerometer Register Addresses
#define Power_Register 0x2D
#define X_Axis_Register_DATAX0 0x32 // Hexadecima address for the DATAX0 internal register.
#define X_Axis_Register_DATAX1 0x33 // Hexadecima address for the DATAX1 internal register.
#define Y_Axis_Register_DATAY0 0x34 
#define Y_Axis_Register_DATAY1 0x35
#define Z_Axis_Register_DATAZ0 0x36
#define Z_Axis_Register_DATAZ1 0x37
int ADXAddress = 0x53;  //Device address in which is also included the 8th bit for selecting the mode, read in this case.
int X0,X1,X_out;
int Y0,Y1,Y_out;
int Z1,Z0,Z_out;
float Xa,Ya,Za;
//--- Gyro Register Addresses
#define Gyro_gX0 0x28  
#define Gyro_gX1 0x29
#define Gyro_gY0 0x2A
#define Gyro_gY1 0x2B
#define Gyro_gZ0 0x2C  
#define Gyro_gZ1 0x2D
int Gyro = 0x69; //Device address in which is also included the 8th bit for selecting the mode, read in this case.
int gX0, gX1, gX_out;
int gY0, gY1, gY_out;
int gZ0, gZ1, gZ_out;
float Xg,Yg,Zg;
float angleX,angleY,angleZ,angleXc,angleYc,angleZc;
unsigned long start, finished, elapsed;
float dt=0.015;
//Magnetometer
#define Magnetometer 0x1E //I2C 7bit address of HMC5883
#define Magnetometer_mX0 0x03  
#define Magnetometer_mX1 0x04  
#define Magnetometer_mZ0 0x05  
#define Magnetometer_mZ1 0x06  
#define Magnetometer_mY0 0x07  
#define Magnetometer_mY1 0x08  
int mX0, mX1, mX_out;
int mY0, mY1, mY_out;
int mZ0, mZ1, mZ_out;
float Xm,Ym,Zm;

float phi=0,theta=0,sih=0;
float phia,thetaa,sihm;
float dphig,dthetag,dsihg;
float alpha=0.8;


float delt=0.01;
float phig=0,thetag=0,sihg=0;

void setup() {
  Wire.begin(); // Initiate the Wire library    
  Serial.begin(9600);    
  delay(100);
  
  Wire.beginTransmission(ADXAddress);
  Wire.write(Power_Register); // Power_CTL Register
  // Enable measurement
  Wire.write(8); // Bit D3 High for measuring enable (0000 1000)
  Wire.endTransmission();

  
  
  Wire.beginTransmission(Gyro);
  Wire.write(0x20); // CTRL_REG1 - Power Mode
  Wire.write(15);   // Normal mode: 15d - 00001111b   
  Wire.endTransmission();
  
  Wire.beginTransmission(Gyro);
  Wire.write(0x23); // CTRL_REG4 - Sensitivity, Scale Selection
  Wire.write(48);   // 2000dps: 48d - 00110000b
  Wire.endTransmission();


   Wire.beginTransmission(Magnetometer); 
  Wire.write(0x02); // Select mode register
  Wire.write(0x00); // Continuous measurement mode
  Wire.endTransmission();
}


void loop()
{
{
  // X-axis
  Wire.beginTransmission(ADXAddress); // Begin transmission to the Sensor 
  //Ask the particular registers for data
  Wire.write(X_Axis_Register_DATAX0);
  Wire.write(X_Axis_Register_DATAX1);  
  Wire.endTransmission(); // Ends the transmission and transmits the data from the two registers
  Wire.requestFrom(ADXAddress,2); // Request the transmitted two bytes from the two registers
  if(Wire.available()<=2) {  // 
    X0 = Wire.read(); // Reads the data from the register
    X1 = Wire.read();
    /* Converting the raw data of the X-Axis into X-Axis Acceleration
     - The output data is Two's complement 
     - X0 as the least significant byte
     - X1 as the most significant byte */ 
    X1=X1<<8;
    X_out =X0+X1;
    Xa=X_out/256.0; // Xa = output value from -1 to +1, Gravity acceleration acting on the X-Axis
  }
  // Y-Axis
  Wire.beginTransmission(ADXAddress); 
  Wire.write(Y_Axis_Register_DATAY0);
  Wire.write(Y_Axis_Register_DATAY1);  
  Wire.endTransmission(); 
  Wire.requestFrom(ADXAddress,2);
  if(Wire.available()<=2) { 
    Y0 = Wire.read();
    Y1 = Wire.read();
    Y1=Y1<<8;
    Y_out =Y0+Y1;
    Ya=Y_out/256.0;
  }
  // Z-Axis
  Wire.beginTransmission(ADXAddress); 
  Wire.write(Z_Axis_Register_DATAZ0);
  Wire.write(Z_Axis_Register_DATAZ1);  
  Wire.endTransmission(); 
  Wire.requestFrom(ADXAddress,2);
  if(Wire.available()<=2) { 
    Z0 = Wire.read();
    Z1 = Wire.read();
    Z1=Z1<<8;
    Z_out =Z0+Z1;
    Za=Z_out/256.0;
  }
  // Prints the data on the Serial Monitor
/*  Serial.print("Xa= ");
  Serial.print(Xa);
  Serial.print("   Ya= ");
  Serial.print(Ya);
  Serial.print("   Za= ");
  Serial.println(Za);*/
}
{
  start=millis();
  //---- X-Axis
  Wire.beginTransmission(Gyro); // transmit to device
  Wire.write(Gyro_gX0);
  Wire.endTransmission();
  Wire.requestFrom(Gyro,1); 
  if(Wire.available()<=1)   
  {
    gX0 = Wire.read();
  }
  Wire.beginTransmission(Gyro); // transmit to device
  Wire.write(Gyro_gX1);
  Wire.endTransmission();
  Wire.requestFrom(Gyro,1); 
  if(Wire.available()<=1)   
  {
    gX1 = Wire.read();
  }
  //---- Y-Axis
  Wire.beginTransmission(Gyro); // transmit to device
  Wire.write(Gyro_gY0);
  Wire.endTransmission();
  Wire.requestFrom(Gyro,1); 
  if(Wire.available()<=1)   
  {
    gY0 = Wire.read();
  }
  Wire.beginTransmission(Gyro); // transmit to device
  Wire.write(Gyro_gY1);
  Wire.endTransmission();
  Wire.requestFrom(Gyro,1); 
  if(Wire.available()<=1)   
  {
    gY1 = Wire.read();
  }
  
  //---- Z-Axis
  Wire.beginTransmission(Gyro); // transmit to device
  Wire.write(Gyro_gZ0);
  Wire.endTransmission();
  Wire.requestFrom(Gyro,1); 
  if(Wire.available()<=1)   
  {
    gZ0 = Wire.read();
  }
  Wire.beginTransmission(Gyro); // transmit to device
  Wire.write(Gyro_gZ1);
  Wire.endTransmission();
  Wire.requestFrom(Gyro,1); 
  if(Wire.available()<=1)   
  {
    gZ1 = Wire.read();
  }
  
  //---------- X - Axis
  
  // Raw Data
  gX1=gX1<<8;
  gX_out =gX0+gX1;
  
  // From the datasheet: 70 mdps/digit
  Xg=gX_out*0.07; // Angular rate
  // Angular_rate * dt = angle
  //angleXc = Xg*dt;
  //angleX = angleX + angleXc;
  //---------- Y - Axis
  gY1=gY1<<8;
  gY_out =gY0+gY1;
  Yg=gY_out*0.07;
  //angleYc = Yg*dt;
 // angleY = angleY + angleYc;
  
  //---------- Z - Axis
  gZ1=gZ1<<8;
  gZ_out =gZ0+gZ1;
  Zg=gZ_out*0.07;
 // angleZc = Zg*dt;
 // angleZ = angleZ + angleZc;
  
  // Prints the data on the Serial Monitor
 /* Serial.print("p= ");
  Serial.print(angleX);
  Serial.print(" q= ");
  Serial.print(angleY);
  Serial.print(" r= ");
  Serial.println(angleZ);*/
  
  delay(100);
  // Calculating dt
  finished=millis();
  elapsed=finished-start;
  dt=elapsed/1000.0;
  start = elapsed = 0;
  
}
{
 
  //---- X-Axis
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mX1);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer,1); 
  if(Wire.available()<=1)   
  {
    mX0 = Wire.read();
  }
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mX0);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer,1); 
  if(Wire.available()<=1)   
  {
    mX1 = Wire.read();
  }
  //---- Y-Axis
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mY1);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer,1); 
  if(Wire.available()<=1)   
  {
    mY0 = Wire.read();
  }
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mY0);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer,1); 
  if(Wire.available()<=1)   
  {
    mY1 = Wire.read();
  }
  
  //---- Z-Axis
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mZ1);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer,1); 
  if(Wire.available()<=1)   
  {
    mZ0 = Wire.read();
  }
  Wire.beginTransmission(Magnetometer); // transmit to device
  Wire.write(Magnetometer_mZ0);
  Wire.endTransmission();
  Wire.requestFrom(Magnetometer,1); 
  if(Wire.available()<=1)   
  {
    mZ1 = Wire.read();
  }
  
  //---- X-Axis
  mX1=mX1<<8;
  mX_out =mX0+mX1; // Raw data
  // From the datasheet: 0.92 mG/digit
  Xm = mX_out*0.00092; // Gauss unit
  //* Earth magnetic field ranges from 0.25 to 0.65 Gauss, so these are the values that we need to get approximately.
  //---- Y-Axis
  mY1=mY1<<8;
  mY_out =mY0+mY1;
  Ym = mY_out*0.00092;
  //---- Z-Axis
  mZ1=mZ1<<8;
  mZ_out =mZ0+mZ1;
  Zm = mZ_out*0.00092;
 
  //Print out values of each axis
 /* Serial.print("Xm: ");
  Serial.print(Xm);
  Serial.print("  Ym: ");
  Serial.print(Ym);
  Serial.print("  Zm: ");
  Serial.println(Zm);*/
  
  delay(100);
}

dphig=angleX+(sin(phi)*tan(theta)*angleY) + (cos(phi)*tan(theta)*angleZ);
dthetag=(angleY*cos(phi)) -( angleZ*sin(phi));
dsihg=(angleY*sin(phi)*(1/cos(theta))) +(angleZ*cos(phi)*(1/cos(theta)));
phig=phig+(dphig*delt);
thetag=thetag+(dthetag*delt);
sihg=sihg+(dsihg*delt);
phia=atan(Ya/Za);
thetaa= atan((-Xa)/((Ya*sin(phi))+(Za*cos(phi))));
sihm=atan(((Zm*sin(phi))-(Ym*cos(phi)))/((Xm*cos(phi))+(Ym*sin(theta)*sin(phi))+(Zm*sin(theta)*cos(phi))));

phi=(alpha*phig)+((1-alpha)*phia);
theta=(alpha*thetag)+((1-alpha)*thetaa);
sih=(alpha*sihg)+((1-alpha)*sihm);

Serial.print(" Roll: ");
Serial.print(phi);
Serial.print(" Pitch: ");
Serial.print(theta);
Serial.print(" Yaw: ");
Serial.println(sih);


}
