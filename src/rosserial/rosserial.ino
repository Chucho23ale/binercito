#include <stdlib.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

 

#define WHEELRAD 0.0636 //The radius of the wheel (m) 

#define WHEELDIST 0.34 //Distance between wheels (m) 



ros::NodeHandle  nh;
std_msgs::Float32 wl_ref_msg, wl_msg, wr_ref_msg, wr_msg;
geometry_msgs::Twist twist;
ros::Publisher wl_refT("wl_ref", &wl_ref_msg);
ros::Publisher wlT("wl", &wl_msg);
ros::Publisher wr_refT("wr_ref", &wr_ref_msg);
ros::Publisher wrT("wr", &wr_msg);



//Encoder #1
const int pinCanalAl=2;
const int pinCanalBl=4;

//Encoder #2
const int pinCanalAr=3;
const int pinCanalBr=5;

//Control Motor 1
const int motorPin1l=10;
const int motorPin2l=11;

//Control Motor 2
const int motorPin1r=12;
const int motorPin2r=13;

float dt=0.02;
float tiempo=0;
float contadorl=0;
float contadorr=0;
float posl=0;
float posr=0;
float pos_antl=0;
float pos_antr=0;


float Ts;
float kpl, kil, kdl;
float kpr, kir, kdr;
float q0l, q1l, q2l;
float q0r, q1r, q2r;
float ul[2] = {0.0, 0.0}; // u[0] salida actual   u[1] salida anterior
float el[3] = {0.0, 0.0, 0.0};  //e[0] error actual    e[1] anterior  e[2] dos veces anterior
float ur[2] = {0.0, 0.0}; // u[0] salida actual   u[1] salida anterior
float er[3] = {0.0, 0.0, 0.0};  //e[0] error actual    e[1] anterior  e[2] dos veces anterior
float wl_ref=0;
float wr_ref=0;
float wl=0;
float wr=0;
float v = 0;
float w = 0;

bool hasMessage = false;

void messageCb(const geometry_msgs::Twist &tw_cb){
  hasMessage = true;
  twist = tw_cb;
}

ros::Subscriber<geometry_msgs::Twist>sub("ard_motor", &messageCb);


//Configuracion de puertos E/S, Serial e interrupciones
void setup() {
  attachInterrupt(digitalPinToInterrupt(pinCanalAl),Encoder,RISING);
  pinMode(pinCanalAl,INPUT);
  pinMode(pinCanalBl,INPUT);

  
  attachInterrupt(digitalPinToInterrupt(pinCanalAr),Encoder2,RISING);
  pinMode(pinCanalAr,INPUT);
  pinMode(pinCanalBr,INPUT);
  
  pinMode(motorPin1l,OUTPUT);
  pinMode(motorPin2l,OUTPUT);

  pinMode(motorPin1r,OUTPUT);
  pinMode(motorPin2r,OUTPUT);
  
//  kpl = 4.4;
//  kil = 30;
//  kdl = 0.01;
//  kpr = 4.4;
//  kir = 30;
//  kdr = 0.01;

  kpl = 1.18;
  kil = 0.12;
  kdl = 0.01;
  kpr = 1.079;
  kir = 0.15;
  kdr = 0.01;
  
  Ts = 0.02; 
 
  q0l= kpl+(kil*Ts/2)+(kdl/Ts);
  q1l= -kpl+(kil*Ts/2)-(2.0*kdl/Ts);
  q2l= kdl/Ts;

  q0r= kpr+(kir*Ts/2)+(kdr/Ts);
  q1r= -kpr+(kir*Ts/2)-(2.0*kdr/Ts);
  q2r= kdr/Ts;

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(wl_refT);
  nh.advertise(wlT);
  nh.advertise(wr_refT);
  nh.advertise(wrT);

}

void loop() {
// Linea de tiempo se usa para controlar el tiempo demuestreo  
  if(hasMessage){
   if(micros()-tiempo>dt*1000000)
   {
    v = twist.linear.x;
    w = twist.angular.z;
    wl_ref = ((2.0 * v - WHEELDIST * w) / (2.0 * WHEELRAD))/(0.10472);
    wr_ref = ((2.0 * v + WHEELDIST * w) / (2.0 * WHEELRAD))/(0.10472);
    
    tiempo=micros();
    posl=contadorl;
    posr=contadorr;
    //Realizar la converción según el motor
    wl=(posl-pos_antl)*2.6786;
    wr=(posr-pos_antr)*2.6786;
  
  if(wl_ref != 0.0){
  //INICIO DE AREA DE CONTROL
      //calcular la señal de error
      el[0] = wl_ref-wl;
      //calcular la ecuación en diferencias
      ul[0]=q0l*el[0]+q1l*el[1]+q2l*el[2]+ul[1];
      //limitar la señal de control
      if(ul[0] > 255.0) ul[0] = 255;
      if(ul[0] < -255.0) ul[0] = -255;
      //corrimiento para mover las muestras
      el[2] = el[1];
      el[1] = el[0];
      ul[1] = ul[0];
  // //FIN DE AREA DE CONTROL
  }
  else{
      el[2] = 0.0;
      el[1] = 0.0;
      el[0] = 0.0;
      ul[1] = 0.0;
      ul[0] = 0.0;
  }
  if(wr_ref != 0.0){
  //INICIO DE AREA DE CONTROL
      //calcular la señal de error
      er[0] = wr_ref-wr;
      //calcular la ecuación en diferencias
      ur[0]=q0r*er[0]+q1r*er[1]+q2r*er[2]+ur[1];
      //limitar la señal de control
      if(ur[0] > 255.0) ur[0] = 255;
      if(ur[0] < -255.0) ur[0] = -255;
      //corrimiento para mover las muestras
      er[2] = er[1];
      er[1] = er[0];
      ur[1] = ur[0];
  // //FIN DE AREA DE CONTROL
  }
  else{
      er[2] = 0.0;
      er[1] = 0.0;
      er[0] = 0.0;
      ur[1] = 0.0;
      ur[0] = 0.0;
  }
    //Enviamos voltaje a puertos salida PWM
    if ((int)ul[0]<0){
      analogWrite(motorPin1l,0);
      analogWrite(motorPin2l,(int)(-ul[0]));
    }
    else if ((int)ul[0]>=0){
      analogWrite(motorPin1l,(int)ul[0]);
      analogWrite(motorPin2l,0);
    }
  
    //Enviamos voltaje a puertos salida PWM
    if ((int)ur[0]<0){
      analogWrite(motorPin1r,0);
      analogWrite(motorPin2r,(int)(-ur[0]));
    }
    else if ((int)ur[0]>=0){
      analogWrite(motorPin1r,(int)ur[0]);
      analogWrite(motorPin2r,0);
    }
    
    wl_ref_msg.data = wl_ref;
    wl_msg.data = ul[0];
    wl_refT.publish(&wl_ref_msg);
    wlT.publish(&wl_msg);
  
    wr_ref_msg.data = wr_ref;
    wr_msg.data = ur[0];
    wr_refT.publish(&wr_ref_msg);
    wrT.publish(&wr_msg);
    
    pos_antl=posl;
    pos_antr=posr;
   }
  }
  nh.spinOnce();
}

//Interrupcion de Encoder 1
void Encoder()
{
  if (digitalRead(pinCanalAl)==digitalRead(pinCanalBl)){
    contadorl--;}
    else{
      contadorl++;}
}

//Interrupcion de Encoder 2
void Encoder2()
{
  if (digitalRead(pinCanalAr)==digitalRead(pinCanalBr)){
    contadorr++;}
    else{
      contadorr--;}
}
