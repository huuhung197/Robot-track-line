#include <LineSensor.h>

//Name-----------Pin


#define led1      13


#define ri1       9
#define ri2       10
#define pwmi      6   //~
#define le1       7
#define le2       8
#define pwmd      11   //~


#define NUM_SENSORS             8
#define NUM_SAMPLES_PER_SENSOR  1
#define EMITTER_PIN             2

LineSensorAnalog Elsa((unsigned char[]) {7, 6, 5, 4, 3, 2, 1, 0}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];
int error = 0;
int diffValue = 50;
int derivative = 0;
int integral = 0;
int exit_pwm = 0;
int error_last = 0;
int position = 0; // Vi Tri
byte LineType = 0;

int maxSpeed = 85; //Toc do toi da

 // float KP=0.15, KD=3.5 , KI=0; // 9v, speed=70;
//  float KP=0.5, KD=5.5 , KI=0; // 12v, speed = 70;
 //float KP = 2.1, KD = 9.5 , KI = 0; //11.98v// speed =100;
//float KP = 0.3, KD = 1.5 , KI = 0;    //11.9V //speed =50
float KP = 0.8, KD = 9 , KI = 0.0001;      //11.93v speed = 200;

int line = 0; // 1 White Line - 0 Black Line
int linea = 0;
int flanco_color =  0 ;
int en_linea =  500 ; //deviation between white and black
int noise = 30; //Noise


unsigned long previousMillis = 0;
unsigned long previousMillis1 = 0;
int timeInterval = 600;
void turn_left(unsigned int t) {

  digitalWrite(le1, LOW);
  digitalWrite(le2, HIGH);
  digitalWrite(ri1, HIGH);
  digitalWrite(ri2, LOW);
  analogWrite(pwmd, 250);
  analogWrite(pwmi, 250);
  delay(t);

}
void turn_right(unsigned int t) {

  digitalWrite(le1, HIGH);
  digitalWrite(le2, LOW);
  digitalWrite(ri1, LOW);
  digitalWrite(ri2, HIGH);
  analogWrite(pwmd, 250);
  analogWrite(pwmi, 250);
  delay(t);

}

void motor_run(int motor_left, int motor_right)
{
  if ( motor_left >= 0 )
  {
    digitalWrite(le1, HIGH);
    digitalWrite(le2, LOW);
    analogWrite(pwmi, motor_left);
  }
  else
  {
    digitalWrite(le1, LOW);
    digitalWrite(le2, HIGH);
    motor_left = motor_left * (-1);
    analogWrite(pwmi, motor_left);
  }

  if ( motor_right >= 0 )
  {
    digitalWrite(ri1, HIGH);
    digitalWrite(ri2, LOW);
    analogWrite(pwmd, motor_right);
  }
  else
  {
    digitalWrite(ri1, LOW);
    digitalWrite(ri2, HIGH);
    motor_right = motor_right * (-1);
    analogWrite(pwmd, motor_right);
  }
}

void dung(unsigned int t) {
  digitalWrite(le1, LOW);
  digitalWrite(le2, LOW);
  digitalWrite(ri1, LOW);
  digitalWrite(ri2, LOW);
  delay(t);
}

void pid(int linea, int maxSpeed, float Kp, float Ki, float Kd)
{  
   position = Elsa.readLine(sensorValues,LS_EMITTERS_ON, linea, flanco_color, en_linea, noise);
  error = (position) - 3500; 
//  if(error >-300 && error <300)
//    {maxSpeed = 200;
//    maxSpeed -= 5;}
//  else
//  {  maxSpeed = 50;
//    maxSpeed += 5;}
 
  integral=integral + error_last;
  derivative = (error - error_last);
  int ITerm=integral*KI;
  if(ITerm>=255) ITerm=255;
  if(ITerm<=-255) ITerm=-255;
   
  exit_pwm =( error * KP ) + ( derivative * KD )+(ITerm);
   
  if (  exit_pwm >maxSpeed )  exit_pwm = maxSpeed;
  if ( exit_pwm  <-maxSpeed )  exit_pwm = -maxSpeed;
   
  if (exit_pwm < 0)
 {
    int right=maxSpeed-exit_pwm; //(+)
    int left=maxSpeed+exit_pwm;  //(-)
    if(right>=255)right=255;
    if(left<=0)left=0;
    motor_run(left, right);
  //  Serial.print(left);Serial.print("\t");Serial.println(right);
 }
 if (exit_pwm >0)
 {
  int right= maxSpeed-exit_pwm; //(-)
  int left= maxSpeed+exit_pwm; //(+)
   
  if(left >= 255) left=255;
  if(right <= 0) right=0;
  motor_run(left ,right );
 //  Serial.print(left);Serial.print("\t");Serial.println(right);
 }
 
 error_last = error;  
}
 



void brakes_contour(int flanco_comparacion)
{
 // Elsa.read(sensorValues);
  if (position <10)
  {
    Elsa.read(sensorValues);
    // flanco_comparacion = int((sensorValues[0] +sensorValues[1])/2);
    while (true)
    {
      digitalWrite(led1, HIGH);
      turn_left(2);
      dung(2);
      Elsa.read(sensorValues); //lectura en bruto de sensor
      if ((sensorValues[7] > (flanco_comparacion + 50)) ||(sensorValues[6] > (flanco_comparacion + 50)) ||(sensorValues[5] > (flanco_comparacion + 50)) ||(sensorValues[4] > (flanco_comparacion + 50)) ||(sensorValues[3] > (flanco_comparacion + 50)) ||(sensorValues[2] > (flanco_comparacion + 50)) ||(sensorValues[1] > (flanco_comparacion + 50)) || (sensorValues[0] > (flanco_comparacion + 50))) break;
    }
  }

  if (position >6900)
  {
    Elsa.read(sensorValues);
    // flanco_comparacion = int((sensorValues[6] +sensorValues[7])/2);
    while (true)
    {
      digitalWrite(led1, HIGH);
      turn_right(2);
      dung(2);
      Elsa.read(sensorValues);
      if ((sensorValues[0] > (flanco_comparacion + 50)) ||(sensorValues[1] > (flanco_comparacion + 50)) ||(sensorValues[2] > (flanco_comparacion + 50)) ||(sensorValues[3] > (flanco_comparacion + 50)) ||(sensorValues[4] > (flanco_comparacion + 50)) ||(sensorValues[5] > (flanco_comparacion + 50)) ||(sensorValues[6] > (flanco_comparacion + 50)) || (sensorValues[7] > (flanco_comparacion + 50))) break;
    }
  }
  digitalWrite(led1, LOW);
}



void setup()
{
  Serial.begin(9600);
  Serial.println("Starting...");
  pinMode(led1, OUTPUT);

  pinMode(le1, OUTPUT);
  pinMode(le2, OUTPUT);
  pinMode(pwmi, OUTPUT);
  pinMode(ri1, OUTPUT);
  pinMode(ri2, OUTPUT);
  pinMode(pwmd, OUTPUT);
  delay(1000);
  Elsa.read(sensorValues);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    Elsa.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }

  digitalWrite(13, LOW);
  digitalWrite(ri1, LOW);
  digitalWrite(ri2, LOW);
  digitalWrite(le1, LOW);
  digitalWrite(le2, LOW);
  for (int i = 0; i < 5; i++) {

    digitalWrite(led1, LOW);     // turn off Arduino's LED to indicate we are through with calibration
    delay(300);
    digitalWrite(led1, HIGH);
    delay(300);
  }
  digitalWrite(led1, LOW);
  delay(1000);
  Serial.println("ket thuc");

}

void loop()
{
//
//     pid(line,maxSpeed,KP,KI,KD);
//   brakes_contour(800);
  int position = Elsa.readLine(sensorValues, LS_EMITTERS_ON, linea, flanco_color, en_linea, noise);
  ////
  //////
  //////
  // Elsa.read(sensorValues);
  //
  for (unsigned char i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  // position = Elsa.readLine(sensorValues,LS_EMITTERS_ON, linea, flanco_color, en_linea, noise);
  //Serial.println(); // uncomment this line if you are using raw values
  Serial.println(position); // comment this line out if you are using raw values

//  delay(250);
  // turn_left(1000);
  // turn_right(1000);
 
//  digitalWrite(le1, HIGH);
//  digitalWrite(le2, LOW);
//  digitalWrite(ri1, HIGH);
//  digitalWrite(ri2, LOW);
//  analogWrite(pwmd, 250);
//  analogWrite(pwmi, 250);
// motor_run(255,255);




}
