/*
                                 Maze solving robot

This sketch implements a simple wall following algorithm using PD controller as a feedback control system.
There are two switchButtons that allows the user to choose which wall to follow as per the maze.
Also the robot can be manually controlled via an android app developed using MIT app inventor.

* two continuous servo motors driven by a 6.6 V external battery pack.
* arduino uno powered by 9 v battery.
* two IR sensros and one sonar sensor to sense the surroundings
* a 5v voltage regulator to regulate the voltage from the 6.6 v battery pack
* RN42 bluetooth module 
* push buttons

created 2015
by Nikesh Lama
Nottingham Trent University
*/


#include <Servo.h>

#define  TRIGPIN  11
#define  ECHOPIN  12

#define left_irPin A0
#define right_irPin A1
// stores the char from the serial read for bluetooth communication
char val;                    
 
int SonarDistance;           
int IrDistance;

int leftIrDistance;
int rightIrDistance;

int left_setpoint = 400; 
int right_setpoint = 380;
int threshold =20;
float error = 0;
//PID parameters
float integral = 0;
float derivative = 0;
float last_error = 0;
float PID_value = 0;
//tuning these parameters is pain in the ass.
//probably could use some optimisation to reach best option but too much work
float Kp = 0.7;
float Ki = 0.005;
float Kd = 1.3;
//These correction speed are purely experimental
//Needs to manually checked to get the appropriate correction speed
int correctionSpeed=54;         
int correctionSpeedR = 53;
int rightWheelSpeed;
int leftWheelSpeed;
int base_speedLeft=30;
int base_speedRight =150;

Servo servoLeft ;
Servo servoRight;

int switchPinleftwall = 3; //push button to activate leftwall following algorithm
int switchPinrightwall = 4; //push button to activate rightwall following algorithm
int ledPin = 13;
boolean lastButton_r = LOW;  //last button state for right pushbutton
boolean currentButton_r = LOW;  //current button state for right pushbutton
boolean lastButton_l = LOW;     //last button state for left pushbutton
boolean currentButton_l = LOW;  //current button state for left pushbutton


void setup()
{
  Serial.begin(9600);
  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);
  pinMode(switchPinleftwall, INPUT);
  //pinMode(switchPinBT,INPUT);
  pinMode(ledPin,OUTPUT);
  servoLeft.attach(6); // left wheel
  servoRight.attach(5); // right wheel

}

void loop()
{
  //checking the current state of the button
  currentButton_r = debounce(lastButton_r,switchPinrightwall);
  currentButton_l = debounce(lastButton_l,switchPinleftwall);  
  
  //if right button is pressed----right wall following
  if(lastButton_r == LOW && currentButton_r == HIGH)
  {
    Serial.println("button Pressed(Motor start)://");
    //keep it running unless the button is pressed again
    while(lastButton_r == LOW && currentButton_r == HIGH)
    {
      reading_sensors(right_irPin); 
      pid_calculation(right_setpoint);
      rightwall_follow();
      //check if the button is pressed, if pressed break the loop
      //This push button debounce is a little buggy, doesnt work sometimes, 2 out of 10 times.
      if(debounce(lastButton_r,switchPinrightwall)==HIGH)break;
    }
  }
  //change the current state as last state for another loop
  lastButton_r = currentButton_r;
  
  //-----left wall following//checking the state of the button press
  if(lastButton_l == LOW && currentButton_l == HIGH)
  {
    Serial.println("button Pressed(Motor start)://");
    while(lastButton_l == LOW && currentButton_l == HIGH)
    {
      reading_sensors(left_irPin); 
      pid_calculation(left_setpoint);
      leftwall_follow();
      if(debounce(lastButton_l,switchPinleftwall)==HIGH)break;
    }
  digitalWrite(ledPin,LOW); 
  lastButton_l = currentButton_l;
  }
  
  
   servoLeft.write(90);
   servoRight.write(90);
   digitalWrite(ledPin,LOW);
  //bluetooth control -- 
  bluetooth_control();
  
}


//-----------------------------------------Functions------------------------------------

//allows to choose whether to read left or right ir sensor
void reading_sensors(int choose_ir_sensor)
{
  int sensor_pin = choose_ir_sensor;             //sensor pin could be left or right IR pin
  digitalWrite(TRIGPIN, LOW);                   // Set the trigger pin to low for 2uS
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);                  // Send a 10uS high to trigger ranging
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);                   // Send pin low again
  int distance = pulseIn(ECHOPIN, HIGH);        // Read in times pulse

  SonarDistance = distance/58;// Calculate distance from time of pulse
  Serial.print("Sonar Sensor: ");
  Serial.println(SonarDistance);                     
  delay(50); 
  
  IrDistance= analogRead(sensor_pin);             //read the sensor
  Serial.print(" IR sensor reading: ") ;
  Serial.println(IrDistance);
  Serial.println("cm");
  delay(50);
  
}

void pid_calculation(int choose_setpoint)          // does the pid calculation, setpoint can be changed either for left or right sensor   
{
  //taking the difference in reading 
  int setpoint = choose_setpoint;
  //IrDistance could be left or right depending on the wall follwing algorithm..
  error= IrDistance - setpoint;
  Serial.println(IrDistance);
  Serial.println(error);
  //calculating integral 
  if(abs(error) < threshold)
  {
    integral = integral + error;
  }
  else
  {
    integral=0;
  }
  //taking the change in difference in reading
  derivative = error - last_error;
  last_error = error;
  //calulating PID value which is also an error value
  PID_value = int(error*Kp + integral*Ki + derivative*Kd);
  //return PID_value;
 // Serial.println(PID_value);
  //delay(1000);
}


void leftwall_follow(){

  //sets the PID_value to be lowest to -45 and highest to 45
  if(PID_value< -correctionSpeed){
    PID_value = -correctionSpeed;
    //Serial.println(PID_value);
    //delay(1000);
  }
  if(PID_value> correctionSpeed){
    PID_value = correctionSpeed;
    //Serial.println(PID_value);
    // delay(1000);
  }

  if(PID_value < 0)
  {
    //away from the wall
    rightWheelSpeed = base_speedRight - PID_value   ;
    leftWheelSpeed = base_speedLeft - PID_value;
  }
  else
  {
    //right turn
    rightWheelSpeed = base_speedRight - PID_value;
    leftWheelSpeed = base_speedLeft - PID_value;
  }

  servoLeft.write(leftWheelSpeed);
  servoRight.write(rightWheelSpeed);
  digitalWrite(ledPin,HIGH);


  //sharp right turn(90 degree turn) when theres a wall on the left and wall at the front as well

  if(SonarDistance <7 && SonarDistance!=0  && IrDistance > 100)
  {
    sharp_right();
  }
}

void sharp_right()
{
  //servoLeft.write(90);
  //servoRight.write(90);
  //digitalWrite(ledPin,LOW);
  //delay(500);
  servoLeft.write(0);
  servoRight.write(0);
  digitalWrite(ledPin,HIGH);
  delay(430);
  //servoLeft.write(90);
  //servoRight.write(90);
  //digitalWrite(ledPin,LOW);
  //delay(500);
}

//Can only use either right or left wall following at one time. No point using both left and right wall follow
//Right wall follow is just here to show that it can follow right wall as well if need be. IT is exactlyt he same as left 
//wall following, nothing fancy
//----------------------------rightwallfollow--------------------------------------------------
void rightwall_follow(){

  
  if(PID_value< -correctionSpeedR){
    PID_value = -correctionSpeedR;
    //Serial.println(PID_value);
    //delay(1000);
  }
  if(PID_value> correctionSpeedR){
    PID_value = correctionSpeedR;
    //Serial.println(PID_value);
    // delay(1000);
  }

  if(PID_value < 0)
  {
    //away from the wall
    leftWheelSpeed = base_speedLeft + PID_value;
    rightWheelSpeed = base_speedRight + PID_value;
  }
  else
  {
    //left turn
    leftWheelSpeed = base_speedLeft + PID_value;
    rightWheelSpeed = base_speedRight + PID_value;
  }

  servoLeft.write(leftWheelSpeed);
  servoRight.write(rightWheelSpeed);
  digitalWrite(ledPin,HIGH);


  //sharp right turn when theres a wall on the left and wall at the front as well

  if(SonarDistance <8 && SonarDistance!=0  && IrDistance > 150)
  {
    sharp_left();
  }
}

void sharp_left()
{
  //servoLeft.write(90);
  //servoRight.write(90);
  //digitalWrite(ledPin,LOW);
  //delay(500);
  servoLeft.write(180);
  servoRight.write(180);
  digitalWrite(ledPin,HIGH);
  delay(480);
  //servoLeft.write(90);
  //servoRight.write(90);
  //digitalWrite(ledPin,LOW);
  //delay(500);
}



void bluetooth_control(){
 lastButton_r = LOW;  
 currentButton_r = LOW;  
 lastButton_l = LOW;     
 currentButton_l = LOW;
  
  if( Serial.available()>0 )       // if data is available to read
  {
    //declared val as char to keep it simple
    val = Serial.read();         // read it and store it in 'val'
  }
  if( val == 'f' )               // if 'f' was received--forward movement
  {
    Serial.println("forward movement activated"); 
    servoLeft.write(10); 
    servoRight.write(180);
    //delay(500);
    digitalWrite(ledPin,HIGH);
  } 
  else if(val == 'b') {            // b is for backward movement   
    servoLeft.write(180);
    servoRight.write(10);   
    digitalWrite(ledPin,HIGH);
  }
  else if(val == 'l')              // l is for 90 degree left movement
  {
    digitalWrite(ledPin,HIGH);
    servoLeft.write(180);
    servoRight.write(180);
    delay(460);
    val = 's';                     //stops after 90 degree movement 
  }
  else if(val == 'r')               // r is for right turn
  {
    digitalWrite(ledPin,HIGH);
    servoLeft.write(0);
    servoRight.write(0);
    delay(410);
    
    val = 's';                     //stops after 90 degree turn
  }
  else if(val == 's')               //s is for stop
  {
    digitalWrite(ledPin,LOW);
    servoLeft.write(90);
    servoRight.write(90);
    delay(480); 
    
  }
  else if(val == 'a')               //left little 
  {
    digitalWrite(ledPin,LOW);
    servoLeft.write(97);
    servoRight.write(97);
    delay(490); 
    
  } 
  else if(val == 'A')               //right little 
  {
    digitalWrite(ledPin,LOW);
    servoLeft.write(84);
    servoRight.write(84);
    delay(490); 
    
  }  
  delay(50);
}

//debounce function gives current state of the button...can input left button or right button as an attribute
boolean debounce (boolean last,int controlPin)
{
  boolean current = digitalRead(controlPin);
  //if the state has changed
  if(last !=current)
  {
    delay(5);
    current = digitalRead(controlPin);
  }
  return current;
}


