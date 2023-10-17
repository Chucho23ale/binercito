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
 // Falta definir pines de los Encoders al Arduino!!!!
 
 
/*
 * Defincion de constantes para  coversor de Twist a PWM
 */
  
#define r 0.0636 // [m]
#define L 0.3462// [m]
#define wr_max 15.75 // [rad/s]
#define wl_max 17.06 // [rad/s]
#define offset_R 29 // [tentativo]
#define offset_L 28 // [tentativo]

float dt=0.02;
float tiempo=0;
float contador=0;
float pos=0;
float pos_ant=0;
float vel=0;
float VMotor=0;
float valorM=0;


float Ts;
float kp, ki, kd;
float q0, q1, q2;
float u[2] = {0.0, 0.0}; // u[0] salida actual   u[1] salida anterior
float e[3] = {0.0, 0.0, 0.0};  //e[0] error actual    e[1] anterior  e[2] dos veces anterior
float referencia=0;
float retro; //retroalimentacion



//Configuracion de puertos E/S, Serial e interrupciones
void setup() {
  attachInterrupt(digitalPinToInterrupt(pinCanalA),Encoder,RISING);
  pinMode(pinCanalA,INPUT);
  pinMode(pinCanalB,INPUT);

  pinMode(motorPin1,OUTPUT);
  pinMode(motorPin2,OUTPUT);
  
  kp = 4.4;
  ki = 30;
  kd = 0.01;
  Ts = 0.02; 
 
  q0= kp+(ki*Ts/2)+(kd/Ts);
  q1= -kp+(ki*Ts/2)-(2.0*kd/Ts);
  q2= kd/Ts;


}

void loop() {
// Linea de tiempo se usa para controlar el tiempo demuestreo  
 if(micros()-tiempo>dt*1000000)
 {
 tiempo=micros();
  pos=contador;
  //Realizar la converción según el motor
  vel=(pos-pos_ant)*1.836;

//INICIO DE AREA DE CONTROL
    retro = vel;
    //calcular la señal de error
    e[0] = referencia-retro;
    //calcular la ecuación en diferencias
    u[0]=q0*e[0]+q1*e[1]+q2*e[2]+u[1];
    //limitar la señal de control
    if(u[0] > 255.0) u[0] = 255;
    if(u[0] < -255.0) u[0] = -255;
    //corrimiento para mover las muestras
    e[2] = e[1];
    e[1] = e[0];
    u[1] = u[0];
// //FIN DE AREA DE CONTROL

  //Enviamos voltaje a puertos salida PWM
  if ((int)u[0]<0){
    analogWrite(motorPin1,0);
    analogWrite(motorPin2,(int)(-u[0]));
  }
  if ((int)u[0]>=0){
    analogWrite(motorPin1,(int)u[0]);
    analogWrite(motorPin2,0);
  }
  pos_ant=pos;
 }
}

//Interrupcion de Encoder 1
void Encoder()
{
  if (digitalRead(pinCanalA)==digitalRead(pinCanalB)){
    contador++;}
    else{
      contador--;}
}

