#include "ros.h"
#include <std_msgs/String.h>
#include<Servo.h>

int hasMsgSeed = 0;
int servoPin = 2;
String seed_trigger = "";

ros::NodeHandle nh;
std_msgs::String auto_seed;
Servo myServo;

void seedCb(const std_msgs::String &auto_seed){
  hasMsgSeed = 1;
  seed_trigger = auto_seed.data;
}

ros::Subscriber<std_msgs::String>subSeed("/string_command", seedCb);


void setup() {
  // put your setup code here, to run once:
  myServo.attach(servoPin);
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(subSeed);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(hasMsgSeed == 1){
    if(seed_trigger == "drop"){
      myServo.write(0);
      delay(50);
      myServo.write(90);
      delay(1000);
      myServo.write(0);
      delay(50);
    }
  }

  nh.spinOnce();
  delay(15);
}
