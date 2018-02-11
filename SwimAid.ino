   /*
 * Thomas Hall
 * Cristina Guzman-Moumtzis
 * 
 * ADXL335
 * note:vcc-->5v ,but ADXL335 Vs is 3.3V
 * The circuit:
 *       5V: VCC
 * analog 1: x-axis
 * analog 2: y-axis
 * analog 3: z-axis
*/

//connect 3.3v to AREF

const int xPin = A1;
const int yPin = A2;
const int zPin = A3;  

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  
  Serial.print(analogRead(xPin)-115);
  Serial.print("\t");
  
  Serial.print(analogRead(yPin)-120);
  Serial.print("\t");
  
  Serial.print(analogRead(zPin)-110);
  Serial.println();

  delay(10);                     
  
}
/*
const int xPin   = A1;
const int yPin   = A2;
const int zPin   = A3;

// variables
int x = 0;
int y = 0;
int z = 0;

void setup() {

  // activating debugging for arduino nano
  Serial.begin(9600);
} // end setup

void loop() {
  x = analogRead(xPin);
  y = analogRead(yPin);
  z = analogRead(zPin);
  // show x, y and z-values
  Serial.print("x = ");
  Serial.print(x);
  Serial.print(" y = ");
  Serial.print(y);
  Serial.print(" z = ");
  Serial.print(z);
  // show angle
  Serial.print(" angle = ");
  Serial.println(constrain(map(x,349,281,0,90),0,90));
} // end loop
*/

/*
#include <GY_61.h>

#define delayTime 1000

GY_61 accel;

void setup() {
    accel = GY_61(A0, A1, A2);
    Serial.begin(9600);
}
void loop() {
  Serial.print("X = ");
  Serial.println(accel.readx());
  Serial.print(" Y = ");
  Serial.println(accel.ready());
  Serial.print(" Z = ");
  Serial.println(accel.readz());
  Serial.print(" Ac. Total");
  Serial.println(accel.acceltol());
  delay(delayTime);
}


// include library for ADXL335
#include <ADXL335.h>

// declare pins
const int xpin = 1;      // x-axis of the accelerometer
const int ypin = 2;      // y-axis
const int zpin = 3;      // z-axis (only on 3-axis models)
const float aref = 3.3;  // 3.3v
ADXL335 accel(xpin, ypin, zpin, aref);

// globals
int delayTime;          // global variable for delay time

void setup(){
  // initialize the serial communications:
  Serial.begin(9600);

  delayTime = 10;
  
  delay(delayTime);
  Serial.println("X,\tY,\tZ,\tRho,\tPhi,\tTheta");
  Serial.println("-----------------------------");
}

void loop(){

  //this is required to update the values
  accel.update();
  
  //this tells us how long the string is
  int string_width;

  float x;
  float y;
  float z;
  
  //for these variables see wikipedia's
  //definition of spherical coordinates
  float rho;
  float phi;
  float theta;  
  
  x = accel.getX();
  y = accel.getY();
  //if the project is laying flat and top up the z axis reads ~1G
  z = accel.getZ();
  rho = accel.getRho();
  phi = accel.getPhi();
  theta = accel.getTheta();
  
  Serial.print(formatFloat(x, 2, &string_width));
  Serial.print(",\t");
  Serial.print(formatFloat(y, 2, &string_width));
  Serial.print(",\t");
  Serial.print(formatFloat(z, 2, &string_width));
  Serial.print(",\t");
  Serial.print(formatFloat(rho, 2, &string_width));
  Serial.print(",\t");
  Serial.print(formatFloat(phi, 2, &string_width));
  Serial.print(",\t");
  Serial.print(formatFloat(theta, 2, &string_width));
  Serial.println("");
  
  delay(delayTime);
}

//this function was taken from my format float library
String formatFloat(double value, int places, int* string_width)
{
  //if value is positive infinity
  if (isinf(value) > 0)
  {
    return "+Inf";
  }
    
  //Arduino does not seem to have negative infinity
  //keeping this code block for reference
  //if value is negative infinity
  if(isinf(value) < 0)
  {
    return "-Inf";
  }
  
  //if value is not a number
  if(isnan(value) > 0)
  {
    return "NaN";
  }
  
  //always include a space for the dot
  int num_width = 1;

  //if the number of decimal places is less than 1
  if (places < 1)
  {
    //set places to 1
    places = 1;
    
    //and truncate the value
    value = (float)((int)value);
  }
  
  //add the places to the right of the decimal
  num_width += places;
  
  //if the value does not contain an integral part  
  if (value < 1.0 && value > -1.0)
  {
    //add one for the integral zero
    num_width++;
  }
  else
  {

    //get the integral part and
    //get the number of places to the left of decimal
    num_width += ((int)log10(abs(value))) + 1;
  }
  //if the value in less than 0
  if (value < 0.0)
  {
    //add a space for the minus sign
    num_width++;
  }
  
  //make a string the size of the number
  //plus 1 for string terminator
  char s[num_width + 1]; 
  
  //put the string terminator at the end
  s[num_width] = '\0';
  
  
  //initalize the array to all zeros
  for (int i = 0; i < num_width; i++)
  {
    s[i] = '0';
  }
  
  //characters that are not changed by 
  //the function below will be zeros
  
  //set the out variable string width
  //lets the caller know what we came up with
  *string_width = num_width;
  
  //use the avr-libc function dtosrtf to format the value
  return String(dtostrf(value,num_width,places,s));  
}
  

  
  float sum = readValues();
  if(k >= BUFFER_SIZE){
    float avg = 0.0;
    for(int i = 0; i < BUFFER_SIZE; i++){
      avg += buf[i];
    }
    avg = avg / BUFFER_SIZE;
    Serial.print("avg:");
    Serial.print(avg);
    Serial.println("\t");
    
    for(int i = 0; i < BUFFER_SIZE; i++){
      buf[i] = 0.0;
    }
    
    k = 0;
  }
  else{
    buf[k] = sum;
    k++;  
  }
  

// read data and perform calculations for arm movement
float readValues(){
  
  int x = analogRead(xpin);  //read from xpin 
  int y = analogRead(ypin);  //read from ypin 
  int z = analogRead(zpin);  //read from zpin

  float fx = (float)((x - 331.5)/65*9.8);
  float fy = (float)((y - 329.5)/68.5*9.8); 
  float fz = (float)((z - 340)/68*9.8);

  return abs(fx) + abs(fy) + abs(fz);
}

// Print function for debug purposses
void printValues(){
  int x = analogRead(xpin);  //read from xpin
  int y = analogRead(ypin);  //read from ypin
  int z = analogRead(zpin);  //read from zpin
 
  float zero_G = 512.0; //ADC is 0~1023  the zero g output equal to Vs/2
                      //ADXL335 power supply by Vs 3.3V
  float scale = 102.3;  //ADXL335330 Sensitivity is 330mv/g
                       //330 * 1024/3.3/1000 

  Serial.print("raw:");
  Serial.print(x);
  Serial.print("\t scaled:");
  Serial.print(((float)x - 331.5)/65*9.8);  //print x value on serial monitor
  Serial.print("\t");
  Serial.print("raw:");
  Serial.print(y);
  Serial.print("\t scaled:");
  Serial.print(((float)y - 329.5)/68.5*9.8);  //print y value on serial monitor
  Serial.print("\t");
  Serial.print("raw:");
  Serial.print(z);
  Serial.print("\t scaled:");
  Serial.print(((float)z - 340)/68*9.8);  //print z value on serial monitor
  Serial.print("\n");

}*/
