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
 #include <std_msgs/Float32.h>
 #include <geometry_msgs/Twist.h>
 
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
 * DEfincion de constantes para  coversor de Twist a PWM
 */
  
  const float r=0.0636; // [m]
  const float L=0.3462;// [m]
  const float wr_max=15.75; // [rad/s]
  const float wl_max=17.06; // [rad/s]
  const float offset_R=29; // [tentativo]
  const float offset_L=28; // [tentativo]

/*
 * Definición de variables a usar
 */
  ros::NodeHandle nh; // ROS Handler
  std_msgs::Float32 str_msg;
  float pwmL=0;       // Valor para PWM Izquierdo
  float pwmR=0;       // Valor para PWM Derecho
  float v_x = 0.0;    // Velocidad linear
  float w_z = 0.0;    // VElocidad angular
  float wr = 0.0;     //Vel angular rueda derecha
  float wl = 0.0;     // Vel angular rueda izq


/*
 * Definicón de funciones ajenas al loop
 */
  // Control de dirección y velocidad motor derecho
  void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
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
  
  //Función callback para motor izquierdo
  void twist_to_PWM_cb(const geometry_msgs::Twist &msg) {
    v_x = msg.linear.x;
    w_z = msg.angular.z;
  }
/*
 * Creación de subcribers
 */
  ros::Subscriber<geometry_msgs::Twist> twist_sub("/ard_motor", twist_to_PWM_cb);
 
  ros::Publisher chatter("chatter", &str_msg);



void setup() {

  //Inicializr comunicación con ROS
  nh.initNode();
  
  //Subscribirse a los tópicos ROS
  nh.subscribe(twist_sub);
  nh.advertise(chatter);
  
  
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
    wr = (2.0*v_x + w_z*L)/(2.0*r);
    wl = (2.0*v_x - w_z*L)/(2.0*r);
    
    pwmR = wr*(255-offset_R)/wr_max + offset_R;
    pwmL = wl*(255-offset_L)/wl_max + offset_L;
  
  
  if (pwmR > offset_R && pwmR <= 255) {
      setMotor(1, (int) pwmR, PWMA, IN1, IN2);
  } else if (pwmR < (-1.0)*offset_R && pwmR >= -255) {
      setMotor(-1, -1*((int) pwmR), PWMA, IN1, IN2);
  } else if(pwmR == offset_R){
      //Stop if received 0
      setMotor(0, 0, PWMA, IN1, IN2);
      
  } else {
    // Error if received a value outisde range other than 0
      setMotor(0, 0, PWMA, IN1, IN2);
      nh.logerror("Invalid data received; make sure to send a value between -255 and 255");
      nh.logerror("The motor will stop");
    
  }
  
  if (pwmL > offset_L && pwmL <= 255) {
      setMotor(1, (int) pwmL, PWMB, IN3, IN4);
  } else if (pwmL < (-1.0)*offset_L && pwmL >= -255) {
      setMotor(-1, -1*((int) pwmL), PWMB, IN3, IN4);
  }  else if(pwmL == offset_L){
      //Stop if received 0
      setMotor(0, 0, PWMB, IN3, IN4);
      
  } else {
    // Error if received a value outisde range other than 0
      setMotor(0, 0, PWMB, IN3, IN4);
      nh.logerror("Invalid data received; make sure to send a v=alue between -255 and 255");
      nh.logerror("The motor will stop");
    
  }
  
  str_msg.data = pwmR;
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(1);
  
}
