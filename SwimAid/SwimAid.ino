/*****************************************************************
SwimAid.ino
Thomas James Hall | tjh2822@rit.edu
Based off of Adafruit LSM9DS1 Breakout Example 

Hardware setup:
  LSM9DS1 --------- Arduino
   SCL ---------- SCL (A5 on older 'Duinos')
   SDA ---------- SDA (A4 on older 'Duinos')
   VDD ------------- 3.3V
   GND ------------- GND
(CSG, CSXM, SDOG, and SDOXM should all be pulled high. 
Jumpers on the breakout board will do this for you.)

LSM9DS1 Breakout Version: 1.0

*****************************************************************/
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define LSM9DS1_M 0x1E    // Would be 0x1C if SDO_M is LOW
#define LSM9DS1_AG  0x6B  // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_CALCULATED
//#define PRINT_RAW
#define PRINT_SPEED 250   // 250 ms between prints
static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract 
// a declination to get a more accurate heading.
#define DECLINATION +1.54 // Declination (degrees) in Rochester, NY

///////////////////
// SwimAid Setup //
///////////////////

float v = 0;            // set initial velocity to 0
float a = 0;            // set initial acceleration to 0
float x = 25;           // set up target distance for pool
int i = 0;              // set up incrementor;
float xf = 0;           // set up x final to 0
float vf = 0;           // set up v final to 0
float dt = 0;           // set up dt to 0

// TODO - what was dt?

void setup() {
  Serial.begin(115200);
  
  // Before initializing the IMU, there are a few settings
  // we may need to adjust. Use the settings struct to set
  // the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress = LSM9DS1_M;
  imu.settings.device.agAddress = LSM9DS1_AG;
  
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin()) {
    Serial.println("Failed to communicate with LSM9DS1.");
    Serial.println("Double-check wiring.");
    Serial.println("Default settings in this sketch will " \
                  "work for an out of the box LSM9DS1 " \
                  "Breakout, but may need to be modified " \
                  "if the board jumpers are.");
    // Spin our wheels while we wait for the IMU to start
    while (1) {
      ;
    }
  }

  // TODO set initial time?
  int t = (int)millis();

  // First time math
  v=x/dt;
  a=v/dt;
  i++;
}

void loop() {
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() ) {
    // To read from the gyroscope, first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() ) {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() ) {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }
  
  if ((lastPrint + PRINT_SPEED) < millis()) {
    printGyro();  // Print "G: gx, gy, gz"
    printAccel(); // Print "A: ax, ay, az"
    printMag();   // Print "M: mx, my, mz"
    // Print the heading and orientation for fun!
    // Call print attitude. The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other.
    printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz);
    Serial.println();
    
    lastPrint = millis(); // Update lastPrint time
  }

  // Runge Katta 4th order derivation
  //t += dt;

  //get time

  // x = xf            #Using the previous position
  // v = vf            #Using the previous velocity
  // a = vf/dt 

  //rk4(x,v,a,dt) 


  //  xr = vf*0.2          #0.2 is a temporary reaction time
  // xd = xp - xf - xr

  // Msgbox(“turn now”)        #Placeholder for the actual stimulus
}

/*
void rk4(x,v,a,dt){
  x1 = x
  v1 = v
  a1 = a(x1,v1,0)   #I’m assuming we have to define this function
  
  x2 = x + 0.5*v1*dt
  v2 = v + 0.5*a1*dt
  a2 = a(x2, v2, dt/2.0)
  
      x3 = x + 0.5*v2*dt
      v3 = v + 0.5*a2*dt
      a3 = a(x3, v3, dt/2.0)
  
      x4 = x + v3*dt
      v4 = v + a3*dt
      a4 = a(x4, v4, dt)
     xf = x + (dt/6.0)*(v1 + 2*v2 + 2*v3 + v4)
     vf = v + (dt/6.0)*(a1 + 2*a2 + 2*a3 + a4)
  
     return xf, vf   #This is for us to check our work, could remove
  
     Def a(x,v,t)
      a = 2(x-(v*t))/(t^2)   #Thoughts about using this equation?
}
*/

void printGyro() {
  // Now we can use the gx, gy, and gz variables as we please.
  // Either print them as raw ADC values, or calculated in DPS.
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcGyro helper function to convert a raw ADC value to
  // DPS. Give the function the value that you want to convert.
  Serial.print(imu.calcGyro(imu.gx), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");
#elif defined PRINT_RAW
  Serial.print(imu.gx);
  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

void printAccel() {  
  // Now we can use the ax, ay, and az variables as we please.
  // Either print them as raw ADC values, or calculated in g's.
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcAccel helper function to convert a raw ADC value to
  // g's. Give the function the value that you want to convert.
  Serial.print(imu.calcAccel(imu.ax), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");
#elif defined PRINT_RAW 
  Serial.print(imu.ax);
  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif

}

void printMag() {  
  // Now we can use the mx, my, and mz variables as we please.
  // Either print them as raw ADC values, or calculated in Gauss.
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  // If you want to print calculated values, you can use the
  // calcMag helper function to convert a raw ADC value to
  // Gauss. Give the function the value that you want to convert.
  Serial.print(imu.calcMag(imu.mx), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");
#elif defined PRINT_RAW
  Serial.print(imu.mx);
  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

// Calculate pitch, roll, and heading.
void printAttitude(float ax, float ay, float az, float mx, float my, float mz) {
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));
  
  float heading;
  if (my == 0) {
    heading = (mx < 0) ? PI : 0;
  }
  else {
    heading = atan2(mx, my);
  }
    
  heading -= DECLINATION * PI / 180;
  
  if (heading > PI) {
    heading -= (2 * PI);
  }
  else if (heading < -PI) {
    heading += (2 * PI);
  }
  else if (heading < 0) {
    heading += 2 * PI;
  }
  
  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  pitch *= 180.0 / PI;
  roll  *= 180.0 / PI;
  
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.println(roll, 2);
  Serial.print("Heading: "); Serial.println(heading, 2);
}


/*
#include <Adafruit_Sensor.h>
#include <Adafruit_RA8875.h>
#include <Arduino_LSM9DS1.h>
#include <Wire.h>
#include <SPI.h>

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// You can also use software SPI
// Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
// Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

int t = 0;          // time
int dt = 200;       // Timestep
int pos = 50;       // position
int velocity = 5;   // velocity
int eulercomp1 = 50;// For comparison with Euler integration
int eulercomp2 = 5; // For comparison with Euler integration

// set up velocity ('v')
// set up acceleration ('a')
// set up distance for pool ('x')
// set up incrementor ('i')
// set up xf, vf, dt

void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}


void setup() 
{
  Serial.begin(115200);

  while (!Serial) {
    delay(1); // will pause Zero, Leonardo, etc until serial console opens
  }
  
  Serial.println("LSM9DS1 data read demo");
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    while (1);
  }
  Serial.println("Found LSM9DS1 9DOF");

  // helper to just set the default scaling we want, see above!
  setupSensor();

  Serial.print("Initial position: "); Serial.print(pos); 
  Serial.print(". Velocity: "); Serial.print(velocity); Serial.println(".");

  // TODO set initial time?

  // 1st time math
  // v=x/dt;
  // a=v/dt;
  // i++;
}

void loop() 
{
  lsm.read();  /* ask it to read in the data */ 

  /* Get a new sensor event 
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp); 

  Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
  Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
  Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

  Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
  Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
  Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");

  Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
  Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
  Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");

 

   

  Serial.println();
  delay(200);
}


 * 


 * /
 */

