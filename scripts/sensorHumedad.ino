#include<Servo.h>
#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;


int ledIndicador = 7; 
int servoPin = 6;
int SensorPin = A0;
int bajar = 0;

int servoPos = 0 ;
Servo myServo;

void messageCb( const std_msgs::Bool& msg){
  if(msg.data){
    bajar = 1;
  } else {
    bajar = 0;
  }
}

ros::Subscriber<std_msgs::Bool> sub("led", &messageCb );

 void setup() {
//  Serial.begin(9600);
  pinMode(ledIndicador,OUTPUT);
  myServo.attach(servoPin);
  nh.initNode();
  nh.subscribe(sub);
 }

 void loop() {  
  myServo.write(servoPos);
  
  if (bajar == 1){
    int humedad = analogRead(SensorPin);
    Serial.println(humedad);
  
    servoPos = 90;
    Serial.println(servoPos); 
    if(servoPos == 90){
      if(humedad>=600)
      {
        digitalWrite(ledIndicador,LOW);
      }
      else
      {
        digitalWrite(ledIndicador,HIGH);
      }
    } 
  }else{
    servoPos = 0;
    }
    nh.spinOnce();
   delay(1000);
 }
