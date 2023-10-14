#include <stdlib.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>


ros::NodeHandle  nh;
std_msgs::Float32 ref_msg;
std_msgs::Float32 ret_msg;
ros::Publisher referenciaT("referencia", &ref_msg);
ros::Publisher retroT("retro", &ret_msg);


//Encoder #1
const int pinCanalA=2;
const int pinCanalB=4;

//Control Motor 1
const int motorPin1=10;
const int motorPin2=11;


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


void messageCb( const std_msgs::String& refstring){
  String refs = refstring.data;
  referencia = refs.toFloat();
}

ros::Subscriber<std_msgs::String> sub("ref", &messageCb );


//Configuracion de puertos E/S, Serial e interrupciones
void setup() {
  Serial.begin(9600);
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

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(referenciaT);
  nh.advertise(retroT);

}

void loop() {
// Linea de tiempo se usa para controlar el tiempo demuestreo  
 if(micros()-tiempo>dt*1000000)
 {
 tiempo=micros();
  pos=contador;
  //Realizar la converción según el motor
  vel=(pos-pos_ant)*2.6786;

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
  ref_msg.data = referencia;
  ret_msg.data = retro;
  referenciaT.publish(&ref_msg);
  retroT.publish(&ret_msg);
  
  pos_ant=pos;
 }
 nh.spinOnce();
 delay(1);
}

//Interrupcion de Encoder 1
void Encoder()
{
  if (digitalRead(pinCanalA)==digitalRead(pinCanalB)){
    contador--;}
    else{
      contador++;}
}
