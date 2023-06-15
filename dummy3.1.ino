#include <SoftwareSerial.h>
#include <Servo.h>
SoftwareSerial BT_Serial(0, 1); // RX, TX


#define enA 12//Enable1 L298 Pin enA 
#define in1 11 //Motor1  L298 Pin in1 
#define in2 10 //Motor1  L298 Pin in1 
#define in3 9 //Motor2  L298 Pin in1 
#define in4 8 //Motor2  L298 Pin in1 
#define enB 7 //Enable2 L298 Pin enB 

//#define servo A4
Servo servo;

#define R_S A0 //ir sensor Right
#define C_S A1 //ir sensor Center
#define L_S A2 //ir sensor Left

#define echo A3    //Echo pin
#define trigger A4 //Trigger pin


long distance;
int set = 20;
//servo  initial
int pos = 0;
int targetPos = 0;
int increment = 1;
int delayTime = 15;

int bt_ir_data; // variable to receive data from the serial port and IRremote
int Speed = 100;  
int mode=0;
int IR_data;

void setup(){ // put your setup code here, to run once

pinMode(R_S, INPUT); // declare if sensor as input 
pinMode(C_S, INPUT); // declare if sensor as input  
pinMode(L_S, INPUT); // declare ir sensor as input

pinMode(echo, INPUT );// declare ultrasonic sensor Echo pin as input
pinMode(trigger, OUTPUT); // declare ultrasonic sensor Trigger pin as Output  

pinMode(enA, OUTPUT); // declare as output for L298 Pin enA 
pinMode(in1, OUTPUT); // declare as output for L298 Pin in1 
pinMode(in2, OUTPUT); // declare as output for L298 Pin in2 
pinMode(in3, OUTPUT); // declare as output for L298 Pin in3   
pinMode(in4, OUTPUT); // declare as output for L298 Pin in4 
pinMode(enB, OUTPUT); // declare as output for L298 Pin enB 


Serial.begin(9600); // start serial communication at 9600bps
BT_Serial.begin(9600); 
servo.attach(4);


}


void loop(){  

if(BT_Serial.available() > 0){  //if some date is sent, reads it and saves in state     
bt_ir_data = BT_Serial.read(); 
Serial.println(bt_ir_data);     
if(bt_ir_data > 20){Speed = bt_ir_data;}      
}



     if(bt_ir_data == 10){mode=0; Stop();}    //Manual Android Application and IR Remote Control Command   
else if(bt_ir_data == 11){forword(); delay(2000); mode=1; Speed=100;} //Auto Line Follower Command
else if(bt_ir_data == 12){mode=3;}  //Auto Dustbin
else if(bt_ir_data == 13){Serial.println("docking"); fullturn(); backword(); delay(2000); bt_ir_data =10;}  //docking

analogWrite(enA, Speed); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
analogWrite(enB, Speed); // Write The Duty Cycle 0 to 255 Enable Pin B for Motor2 Speed 

if(mode==0){     
//===============================================================================
//                          Key Control Command
//=============================================================================== 
     if(bt_ir_data == 1){forword(); }  // if the bt_data is '1' the DC motor will go forward
else if(bt_ir_data == 2){backword();}  // if the bt_data is '2' the motor will Reverse
else if(bt_ir_data == 3){turnLeft();}  // if the bt_data is '3' the motor will turn left
else if(bt_ir_data == 4){turnRight();} // if the bt_data is '4' the motor will turn right
else if(bt_ir_data == 5){Stop(); }     // if the bt_data '5' the motor will Stop
else if(bt_ir_data == 6){Start();} // if the bt_data is '6' the motor will start for 2 sec
else if(bt_ir_data == 7){fullturn();} // if the bt_data is '4' the motor will turn right




//===============================================================================
//                          Voice Control Command
//===============================================================================    
else if(bt_ir_data == 8){turnLeft();  delay(400);  bt_ir_data = 5;}
else if(bt_ir_data == 9){turnRight(); delay(400);  bt_ir_data = 5;}
}

if(mode==1){    
//===============================================================================
//                          Line Follower Control
//===============================================================================     
if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 0)){forword();}  //if Right Sensor and Left Sensor are at White color then it will call forword function
if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 0)){turnRight();}//if Right Sensor is Black and Left Sensor is White then it will call turn Right function  
if((digitalRead(R_S) == 0)&&(digitalRead(L_S) == 1)){turnLeft();} //if Right Sensor is White and Left Sensor is Black then it will call turn Left function
if((digitalRead(R_S) == 1)&&(digitalRead(L_S) == 1)){Stop(); mode =2;}//if Right Sensor and Left Sensor are at Black color then it will call Stop function
} 

if(mode==2){    
//===============================================================================
//                          Obstacle Avoiding Control
//===============================================================================     
 distance_F = 0;
 distance_F = Ultrasonic_read();
 
  if (distance_F < set){servo_open();}
  else if (distance_F > set){servo_close();}
    
}

if(mode==3){
//===============================================================================
//                          Returning command
//==============================================================================
Serial.println("returning mode");

  turnRight();
  delay(1400);
  forword();
  delay(500);
  bt_ir_data = 11;
  
 
}

/*if(mode==4){
//===============================================================================
//                          Docking control
//==============================================================================
Serial.println("Docking mode");
fullturn();
Stop();
backword();
delay(2000);

}
return;*/
delay(10);
}





//**********************Ultrasonic_read****************************
long Ultrasonic_read(){
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  distance = pulseIn (echo, HIGH);
  return distance / 29 / 2;
}

void servo_open(){
  Serial.println("servo open");
  for (pos = 0; pos <= 90; pos += increment){
    servo.write(pos);
    delay(delayTime)
  }
}

void servo_close(){
  Serial.println("close");
  for (pos = 90; pos >= 0; pos -= increment){
    servo.write(pos);
    delay(delayTime);
  } 
  }

void forword(){  //forword
Serial.println("forward");
digitalWrite(in1, HIGH); //Right Motor forword Pin 
digitalWrite(in2, LOW);  //Right Motor backword Pin 
digitalWrite(in3, LOW);  //Left Motor backword Pin 
digitalWrite(in4, HIGH); //Left Motor forword Pin 
}

void backword(){ //backword
  Serial.println("backward");
digitalWrite(in1, LOW);  //Right Motor forword Pin 
digitalWrite(in2, HIGH); //Right Motor backword Pin 
digitalWrite(in3, HIGH); //Left Motor backword Pin 
digitalWrite(in4, LOW);  //Left Motor forword Pin 
}

void turnRight(){ //turnRight
Serial.println("right");
digitalWrite(in2, LOW);  //Right Motor forword Pin 
digitalWrite(in1, HIGH); //Right Motor backword Pin  
digitalWrite(in4, LOW);  //Left Motor backword Pin 
digitalWrite(in3, HIGH); //Left Motor forword Pin 
}

void turnLeft(){ //turnLeft
Serial.println("left");
digitalWrite(in2, HIGH); //Right Motor forword Pin 
digitalWrite(in1, LOW);  //Right Motor backword Pin 
digitalWrite(in4, HIGH); //Left Motor backword Pin 
digitalWrite(in3, LOW);  //Left Motor forword Pin 
}

void Stop(){ //stop
Serial.println("stop");
digitalWrite(in1, LOW); //Right Motor forword Pin 
digitalWrite(in2, LOW); //Right Motor backword Pin 
digitalWrite(in3, LOW); //Left Motor backword Pin 
digitalWrite(in4, LOW); //Left Motor forword Pin 
}

void Start(){ //start
Serial.println("start"); 
digitalWrite(in1, HIGH); //Right Motor forword Pin 
digitalWrite(in2, LOW); //Right Motor backword Pin 
digitalWrite(in3, HIGH); //Left Motor backword Pin 
digitalWrite(in4, LOW); //Left Motor forword Pin
mode = 1;
}

void fullturn(){ //full turn
Serial.println("full turn");
digitalWrite(in1, LOW);  //Right Motor forword Pin 
digitalWrite(in2, HIGH); //Right Motor backword Pin  
digitalWrite(in3, LOW);  //Left Motor backword Pin 
digitalWrite(in4, HIGH); //Left Motor forword Pin
delay(1000);
}
