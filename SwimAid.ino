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

// declare pins
const int xpin = 1;      // x-axis of the accelerometer
const int ypin = 2;      // y-axis
const int zpin = 3;      // z-axis (only on 3-axis models)

// defines 
#define BUFFER_SIZE 3   // size of our buffer of input data

// globals
int delayTime;          // global variable for delay time
int k;                  // counter for buffer
float buf[BUFFER_SIZE]; // buffer of input data

void setup(){
  // initialize the serial communications:
  Serial.begin(9600);

  delayTime = 1000;
  k = 0;
  
  for(int i = 0; i < BUFFER_SIZE; i++){
    buf[i] = 0.0;
  }

  delay(delayTime);
}

void loop(){
  printValues();

  /*
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
  */
  delay(delayTime);
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

  Serial.print(((float)x - 331.5)/65*9.8);  //print x value on serial monitor
  Serial.print("\t");
  Serial.print(((float)y - 329.5)/68.5*9.8);  //print y value on serial monitor
  Serial.print("\t");
  Serial.print(((float)z - 340)/68*9.8);  //print z value on serial monitor
  Serial.print("\n");

}
