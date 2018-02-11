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
const int xPin = A1;  // our x pin
const int yPin = A2;  // our y pin
const int zPin = A3;  // our x pin

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
