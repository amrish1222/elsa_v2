
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
//#include <QuadratureEncoder.h>
#include <Encoder.h>

#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
#include "odometry.h"


// Odometry Params and Object 
#define ODO_PERIOD 100  // Millis between /tf and /odom publication

const float ticks_per_meter = 5729.58735;
const float meterPerTick = 0.00017453;  
const float base_width   = 0.21;         // Woodie
const float base_length = 0.165;

//long encoderLeftLastValue  = 0L;
//long encoderRightLastValue = 0L;

static unsigned long NextPubMillis   = 0;
static long encoderLeftFrontLastValueOdo  = 0;
static long encoderRightFrontLastValueOdo = 0;
static long encoderLeftBackLastValueOdo  = 0;
static long encoderRightBackLastValueOdo = 0;
static long timeLastOdo    = 0;
float distLeftFront;
float distRightFront;
float distLeftBack;
float distRightBack;

Odometer odo(meterPerTick, base_width, base_length);


// Package Params
bool isHolonomic = true;

// Initialize PID paramaters

double Setpoint_fl, Input_fl, Output_fl;
double Setpoint_fr, Input_fr, Output_fr;
double Setpoint_bl, Input_bl, Output_bl;
double Setpoint_br, Input_br, Output_br;

double aggKp=450, aggKi=250, aggKd=4;
double consKp=280, consKi=120, consKd=1;

PID myPID_fl(&Input_fl, &Output_fl, &Setpoint_fl, aggKp, aggKi, aggKd, DIRECT);
PID myPID_fr(&Input_fr, &Output_fr, &Setpoint_fr, aggKp, aggKi, aggKd, DIRECT);
PID myPID_bl(&Input_bl, &Output_bl, &Setpoint_bl, aggKp, aggKi, aggKd, DIRECT);
PID myPID_br(&Input_br, &Output_br, &Setpoint_br, aggKp, aggKi, aggKd, DIRECT);

// Initialize quadrature encoder paramaters

//Encoders encoder_fleft(2,3); // encoder pins for front left motor A/B
//
//Encoders encoder_bleft(20,21);  // encoder pins for back left motor A/B
//
//Encoders encoder_fright(18,19);  // encoder pins for front right motor A/B
//
//Encoders encoder_bright(A10,A11);  // encoder pins for back right motor A/B


Encoder encoder_fleft(2,3); // encoder pins for front left motor A/B

Encoder encoder_bleft(4,5);  // encoder pins for back left motor A/B

Encoder encoder_fright(23,22);  // encoder pins for front right motor A/B

Encoder encoder_bright(21,20);  // encoder pins for back right motor A/B


// Initialize pin numbers

//const uint8_t LF_PWM = 7;
//const uint8_t LF_FORW = 30;
//const uint8_t LF_BACK = 31;
//
//const uint8_t LB_PWM = 10;
//const uint8_t LB_FORW = 50;
//const uint8_t LB_BACK = 51;
//
//const uint8_t RF_PWM = 9;
//const uint8_t RF_FORW = 34;
//const uint8_t RF_BACK = 35;
//
//const uint8_t RB_PWM = 11;
//const uint8_t RB_FORW = 52;
//const uint8_t RB_BACK = 53;


const uint8_t LF_PWM = 6;
const uint8_t LF_FORW = 7;
const uint8_t LF_BACK = 8;

const uint8_t LB_PWM = 9;
const uint8_t LB_FORW = 10;
const uint8_t LB_BACK = 11;

const uint8_t RF_PWM = 19;
const uint8_t RF_FORW = 18;
const uint8_t RF_BACK = 17;

const uint8_t RB_PWM = 13;
const uint8_t RB_FORW = 15;
const uint8_t RB_BACK = 14;


// speed = 0? help? pid not reset?
bool wtf;
int ticks_since_target = 0;


// Initialize ROS paramaters

ros::NodeHandle nh;

std_msgs::Int32 lfcount;
std_msgs::Int32 rfcount;
std_msgs::Int32 lbcount;
std_msgs::Int32 rbcount;

ros::Publisher lfwheel("lfwheel", &lfcount);
ros::Publisher rfwheel("rfwheel", &rfcount);
ros::Publisher lbwheel("lbwheel", &lbcount);
ros::Publisher rbwheel("rbwheel", &rbcount);

// Cmd_vel Callback
// Sets the setpoints of the pid for each wheel

void onTwist(const geometry_msgs::Twist &msg)
{
  //nh.loginfo("Inside Callback");
  float x = msg.linear.x;
  float y = msg.linear.y;
  float z = msg.angular.z;
  if(x > 0.3){x = 0.3;}
  if(z > 0.25){z = 0.2;}
  float w = 0.2;

  float R = 0.04;
  float L1 = 0.105;
  float L2 = 0.0825;
  float L = (L1+L2);   
  if (!isHolonomic){
    if(!(x==0 && z==0)){
    wtf=false;
      Setpoint_fr = x + (z * w / 2.0)/0.1;
      Setpoint_fl = x - (z * w / 2.0)/0.1;
      Setpoint_br = x + (z * w / 2.0)/0.1;
      Setpoint_bl = x - (z * w / 2.0)/0.1;
    }
    else{
      wtf=true;
      Setpoint_fl = 0;
      Setpoint_fr = 0;
      Setpoint_bl = 0;
      Setpoint_br = 0;
      
    }
  }
  else{
    if(!(x==0 && y==0 && z==0)){
    wtf=false;
      Setpoint_fl = (1/R)*(x - y - L * z);
      Setpoint_fr = (1/R)*(x + y + L * z);
      Setpoint_bl = (1/R)*(x + y - L * z);
      Setpoint_br = (1/R)*(x - y + L * z);
      
    }
    else{
      wtf=true;
      Setpoint_fl = 0;
      Setpoint_fr = 0;
      Setpoint_bl = 0;
      Setpoint_br = 0;
      
    }
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &onTwist);

// Move any motor function with speed_pwm value and pin numbers

void Move_motor(int speed_pwm,const uint8_t pwm,const uint8_t forw,const uint8_t back)
{
  if(speed_pwm >= 0)
  {
    digitalWrite(forw, HIGH);
    digitalWrite(back, LOW);
    analogWrite(pwm, abs(speed_pwm));
  }
  else if(speed_pwm < 0)
  {
    digitalWrite(forw, LOW);
    digitalWrite(back, HIGH);
    analogWrite(pwm, abs(speed_pwm));
  }
}



// Initialize pins for forward movement

void setpins()
{
  pinMode(LF_FORW,OUTPUT);
  pinMode(LF_BACK,OUTPUT);
  pinMode(RF_FORW,OUTPUT);
  pinMode(RF_BACK,OUTPUT);
  pinMode(LF_PWM,OUTPUT);
  pinMode(RF_PWM,OUTPUT);
  pinMode(LB_FORW,OUTPUT);
  pinMode(LB_BACK,OUTPUT);
  pinMode(RB_FORW,OUTPUT);
  pinMode(RB_BACK,OUTPUT);
  pinMode(LB_PWM,OUTPUT);
  pinMode(RB_PWM,OUTPUT);

  digitalWrite(RF_FORW, HIGH);
  digitalWrite(RF_BACK, LOW);
  digitalWrite(LF_FORW, HIGH);
  digitalWrite(LF_BACK, LOW);
  digitalWrite(RB_FORW, HIGH);
  digitalWrite(RB_BACK, LOW);
  digitalWrite(LB_FORW, HIGH);
  digitalWrite(LB_BACK, LOW);
}

// Encoders tend to reverse regarding of the pins??
// This way we move the robot forward a bit on startup
// And if an encoder has negative value we reverse it.

void fix_encoder_ori_on_start(){

  analogWrite(RF_PWM, 180);
  analogWrite(LF_PWM, 180);
  analogWrite(RB_PWM, 180);
  analogWrite(LB_PWM, 180);
  
  delay(150);

  analogWrite(RF_PWM, 0);
  analogWrite(LF_PWM, 0);
  analogWrite(RB_PWM, 0);
  analogWrite(LB_PWM, 0);

//  int ct1 = encoder_fleft.getEncoderCount();
//  int ct2 = encoder_fright.getEncoderCount();
//  int ct3 = encoder_bleft.getEncoderCount();
//  int ct4 = encoder_bright.getEncoderCount();

  int ct1 = encoder_fleft.read();
  int ct2 = encoder_fright.read();
  int ct3 = encoder_bleft.read();
  int ct4 = encoder_bright.read();

//  if(ct1 < 0) {println("reverse front left A/B pins");}
//  if(ct2 < 0) {println("reverse front right A/B pins");}
//  if(ct3 < 0) {println("reverse back left A/B pins");}
//  if(ct4 < 0) {println("reverse back right A/B pins");}
  
}

//void reset Integral error when we stop
void reset_pid_Ki()
{
  myPID_fl.SetMode(MANUAL);
  myPID_fr.SetMode(MANUAL);
  myPID_bl.SetMode(MANUAL);
  myPID_br.SetMode(MANUAL);
  Output_fl=0;
  Output_fr=0;
  Output_bl=0;
  Output_br=0;
  //myPID_fl.SetTunings(aggKp, 0, aggKd);
  //myPID_fr.SetTunings(aggKp, 0, aggKd);
  //myPID_bl.SetTunings(aggKp, 0, aggKd);
  //myPID_br.SetTunings(aggKp, 0, aggKd);
  myPID_fl.SetMode(AUTOMATIC);
  myPID_fr.SetMode(AUTOMATIC);
  myPID_bl.SetMode(AUTOMATIC);
  myPID_br.SetMode(AUTOMATIC);
  //myPID_fl.SetTunings(aggKp, aggKi, aggKd);
  //myPID_fr.SetTunings(aggKp, aggKi, aggKd);
  //myPID_bl.SetTunings(aggKp, aggKi, aggKd);
  //myPID_br.SetTunings(aggKp, aggKi, aggKd);
}

// stop movement

void stop()
{
  digitalWrite(LF_FORW, 0);
  digitalWrite(LF_BACK, 0);
  digitalWrite(RF_FORW, 0);
  digitalWrite(RF_BACK, 0);
  analogWrite(LF_PWM, 0);
  analogWrite(RF_PWM, 0);
  digitalWrite(LB_FORW, 0);
  digitalWrite(LB_BACK, 0);
  digitalWrite(RB_FORW, 0);
  digitalWrite(RB_BACK, 0);
  analogWrite(LB_PWM, 0);
  analogWrite(RB_PWM, 0);

}

void setup() {
  // 115200 baud rate
  nh.getHardware()->setBaud(57600);
  nh.loginfo("Program info");
  // Pid setup
  
  myPID_fl.SetOutputLimits(-255, 255);
  myPID_fr.SetOutputLimits(-255, 255);
  myPID_bl.SetOutputLimits(-255, 255);
  myPID_br.SetOutputLimits(-255, 255);

  myPID_fl.SetMode(AUTOMATIC);
  myPID_fr.SetMode(AUTOMATIC);
  myPID_bl.SetMode(AUTOMATIC);
  myPID_br.SetMode(AUTOMATIC);

  myPID_fl.SetSampleTime(20);
  myPID_fr.SetSampleTime(20);
  myPID_bl.SetSampleTime(20);
  myPID_br.SetSampleTime(20);
  
  // setup pins and fix encoders
   
  setpins();
  // fix_encoder_ori_on_start();

  stop();

  // ros node setup
  
  nh.initNode();
  nh.advertise(lfwheel);
  nh.advertise(rfwheel);
  nh.advertise(lbwheel);
  nh.advertise(rbwheel);
  nh.subscribe(sub);

  odo.setupPubs(nh);

}

// Initialize starting loop paramaters for calculating velocity and time

unsigned long prev = 0;
int old_ct1=0;
int old_ct2=0;
int old_ct3=0;
int old_ct4=0;

void loop() {
  
  // count encoder ticks
//  int ct1 = encoder_fleft.getEncoderCount();
//  int ct2 = encoder_fright.getEncoderCount();
//  int ct3 = encoder_bleft.getEncoderCount();
//  int ct4 = encoder_bright.getEncoderCount();
  int ct1 = encoder_fleft.read();
  int ct2 = encoder_fright.read();
  int ct3 = encoder_bleft.read();
  int ct4 = encoder_bright.read();

  // for some reason if i omit this it does not work properly
  if (ct1!=-1){
    lfcount.data = ct1;}
  if (ct2!=-1){
    rfcount.data = ct2;}
  if (ct3!=-1){
    lbcount.data = ct3;}
  if (ct4!=-1){
    rbcount.data = ct4;}
  // Publish encoder ticks to calculate odom on Jetson Nano side
  lfwheel.publish(&lfcount);
  rfwheel.publish(&rfcount);
  lbwheel.publish(&lbcount);
  rbwheel.publish(&rbcount);

 // calculate time and current velocity
  
  unsigned long now = millis();
  Input_fl = (float(ct1 - old_ct1) / ticks_per_meter) / ((now - prev) / 1000.0);
  Input_fr = (float(ct2 - old_ct2) / ticks_per_meter) / ((now - prev) / 1000.0);
  Input_bl = (float(ct3 - old_ct3) / ticks_per_meter) / ((now - prev) / 1000.0);
  Input_br = (float(ct4 - old_ct4) / ticks_per_meter) / ((now - prev) / 1000.0);
 
  // Use aggresive pid paramaters if gap > 0.1 else conservative
  bool gap1 = abs(Setpoint_fl - Input_fl) < 0.12;
  bool gap2 = abs(Setpoint_fr - Input_fr) < 0.12;
  bool gap3 = abs(Setpoint_bl - Input_bl) < 0.12;
  bool gap4 = abs(Setpoint_br - Input_br) < 0.12;
  
  if(gap1 && gap2 && gap3 && gap4){
    myPID_fl.SetTunings(consKp, consKi, consKd);
    myPID_fr.SetTunings(consKp, consKi, consKd);
    myPID_bl.SetTunings(consKp, consKi, consKd);
    myPID_br.SetTunings(consKp, consKi, consKd);

  }
  else{
    myPID_fl.SetTunings(aggKp, aggKi, aggKd);
    myPID_fr.SetTunings(aggKp, aggKi, aggKd);
    myPID_bl.SetTunings(aggKp, aggKi, aggKd);
    myPID_br.SetTunings(aggKp, aggKi, aggKd);
  }
  if(wtf){
   reset_pid_Ki(); 
  }
  // Compute  Pid
  myPID_fl.Compute();
  myPID_fr.Compute();
  myPID_bl.Compute();
  myPID_br.Compute();




  // Move the motors with the output of the pid
  
  Move_motor(Output_fl,LF_PWM,LF_FORW,LF_BACK);
  Move_motor(Output_fr,RF_PWM,RF_FORW,RF_BACK);
  Move_motor(Output_bl,LB_PWM,LB_FORW,LB_BACK);
  Move_motor(Output_br,RB_PWM,RB_FORW,RB_BACK);


  //==========> OdometryPublsher
  //
  // Check if it is time to publish /odom and /tf

  long encLeftFront = ct1;
  long encRightFront = ct2;
  long encLeftBack = ct3;
  long encRightBack = ct4;
  
  if (now >= NextPubMillis) {
    NextPubMillis = now + ODO_PERIOD;

    // Figure out how far we have gone in meters from last PID computation
    distLeftFront  = meterPerTick * float(encLeftFront  - encoderLeftFrontLastValueOdo);
    distRightFront = meterPerTick * float(encRightFront - encoderRightFrontLastValueOdo);
    distLeftBack  = meterPerTick * float(encLeftBack  - encoderLeftBackLastValueOdo);
    distRightBack = meterPerTick * float(encRightBack - encoderRightBackLastValueOdo);

  
    // Blink the LED to show we are alive
    //toggleLED();
  
    // Publish odometry
    float odoInterval = float(now - timeLastOdo) / 1000.0;
    odo.update_publish(nh.now(), odoInterval, distLeftFront, distRightFront, distLeftBack, distRightBack);

    encoderLeftFrontLastValueOdo  = encLeftFront;
    encoderRightFrontLastValueOdo = encRightFront;
    encoderLeftBackLastValueOdo = encLeftBack;
    encoderRightBackLastValueOdo = encRightBack;
    
    timeLastOdo = now;
    }
 
  // spin the ros node
  
  nh.spinOnce();
  // take the old encoder ticks and time for calculating velocity
//  old_ct1 = encoder_fleft.getEncoderCount();
//  old_ct2 = encoder_fright.getEncoderCount();
//  old_ct3 = encoder_bleft.getEncoderCount();
//  old_ct4 = encoder_bright.getEncoderCount();
  old_ct1 = encoder_fleft.read();
  old_ct2 = encoder_fright.read();
  old_ct3 = encoder_bleft.read();
  old_ct4 = encoder_bright.read();

  prev = now;

  
  delay(25);

}
