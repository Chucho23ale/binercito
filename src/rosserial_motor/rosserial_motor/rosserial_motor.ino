/*
 * Este programa de arduino crea un nodo de rosserial.
 * Controla la velocidad PWM de dos motores:
 * 70:1 Metal Gearmotor 37Dx70L mm 12V with 64 CPR Encoder (Helical Pinion)
 * Se subscribe a dos topicos de ros (pwm_left y pwm_right).
 * Cada tópico recibe un valor de -255 a 255, y respecto a eso
 * envía las señales de control a cada motor para la dirección y velocidad.
 */

 #include "Arduino.h"
 #include <ros.h>
 #include <std_msgs/Int32.h>


 /*
  * Definición de pines a puente H
  */

  #define PWMA 10
  #define PWMB 5
  #define IN2 12
  #define IN1 11
  #define IN3 7
  #define IN4 6

/*
 * Definición de variables a usar
 */
  ros::NodeHandle nh; // ROS Handler
  float pwmL=0;       // Valor para PWM Izquierdo
  float pwmR=0;       // Valor para PWM Derecho



/*
 * Definicón de funciones ajenas al loop
 */
  // Control de dirección y velocidad motor derecho
  void setMotorRight(int dir, int pwmVal, int pwm, int in1, int in2){
    analogWrite(pwm,pwmVal); // Asignación de velocidad
    // Asignación de dirección
    if(dir == -1){       // Atras
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
    }       
    else if(dir == 1){ // Adelant
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
    }
    else{               // Estático
      digitalWrite(in1,LOW);
      digitalWrite(in2,LOW);
    }
  }
  // Control de dirección y velocidad motor derecho
  void setMotorLeft(int dir, int pwmVal, int pwm, int in3, int in4){
    analogWrite(pwm,pwmVal);
  
    if(dir == -1){
      digitalWrite(in3,HIGH);
      digitalWrite(in4,LOW);
    }
    else if(dir == 1){
      digitalWrite(in3,LOW);
      digitalWrite(in4,HIGH);
    }
    else{
      digitalWrite(in3,LOW);
      digitalWrite(in4,LOW);
    }
  }
  
  //Función callback para motor izquierdo
  void pwmLeft_cb(const std_msgs::Int32 & msg) {
          pwmL = msg.data;
          if (pwmL >= 0 && pwmL <= 255) {
                  setMotorLeft(1, (int) pwmL, PWMB, IN3, IN4);
          } else if (pwmL < 0 && pwmL >= -255) {
                  setMotorLeft(-1, -1*((int) pwmL), PWMB, IN3, IN4);
          } else {
                  //Stop if received a wrong value or 0
                  nh.logerror("Invalid data received; make sure to send a value between -255 and 255");
                  nh.logerror("The motor will stop");
                  setMotorLeft(0, 0, PWMB, IN3, IN4);
          }
          digitalWrite(LED_BUILTIN, HIGH);
          delay(100);
          digitalWrite(LED_BUILTIN, LOW);
          delay(100);

  }
  //Función callback para motor derecho
  void pwmRight_cb(const std_msgs::Int32 & msg) {
          pwmR = msg.data;
          if (pwmR >= 0 && pwmR <= 255) {
                  setMotorRight(1, (int) pwmR, PWMA, IN1, IN2);
          } else if (pwmR < 0 && pwmR >= -255) {
                  setMotorRight(-1, -1*((int) pwmR), PWMA, IN1, IN2);
          } else {
                  //Stop if received a wrong value or 0
                  nh.logerror("Invalid data received; make sure to send a value between -255 and 255");
                  nh.logerror("The motor will stop");
                  setMotorRight(0, 0, PWMA, IN1, IN2);
          }
          digitalWrite(LED_BUILTIN, HIGH);
          delay(1000);
          digitalWrite(LED_BUILTIN, LOW);
          delay(1000);
  }

/*
 * Creación de subcribers
 */
  ros::Subscriber<std_msgs::Int32> pwmL_sub("/pwmLeft", pwmLeft_cb);
  ros::Subscriber<std_msgs::Int32> pwmR_sub("/pwmRight", pwmRight_cb);




void setup() {

  //Inicializr comunicación con ROS
  nh.initNode();
  
  //Subscribirse a los tópicos ROS
  nh.subscribe(pwmL_sub);
  nh.subscribe(pwmR_sub);
  
  
  //Declaración de outputs
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  // Declaración de inputs
  // Por el momento no hay
}

void loop() {
  
  nh.spinOnce();
  delay(1);

}
