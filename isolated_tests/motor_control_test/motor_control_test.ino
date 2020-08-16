#include <ros.h>

#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// Initialize pin numbers

const uint8_t LF_PWM = 7;
const uint8_t LF_BACK = 31;
const uint8_t LF_FORW = 30;

const uint8_t LB_BACK = 51;
const uint8_t LB_FORW = 50;
const uint8_t LB_PWM = 10;

const uint8_t RF_PWM = 9;
const uint8_t RF_BACK = 35;
const uint8_t RF_FORW = 34;

const uint8_t RB_PWM = 11;
const uint8_t RB_BACK = 53;
const uint8_t RB_FORW = 52;


double Output_fl=0;
double Output_fr=0;
double Output_bl=0;
double Output_br=0;

// to store current subsciption trigger
unsigned long now = 0;
// to store previous subsciption trigger
unsigned long prev = 0;

int a = 0;

void onTwist(const geometry_msgs::Twist& msg)
{
  a = 1;
  
  prev = millis();
  float x = msg.linear.x;
  float z = msg.angular.z;
  if(x > 0.3){x = 0.3;}
  if(z > 0.25){z = 0.2;}
  float w = 0.2;
  if(!(x==0 && z==0)){
    Output_fr = x + (z * w / 2.0)/0.1;
    Output_fl = x - (z * w / 2.0)/0.1;
    Output_br = x + (z * w / 2.0)/0.1;
    Output_bl = x - (z * w / 2.0)/0.1;
  }
  else{
    Output_fl=0;
    Output_fr=0;
    Output_bl=0;
    Output_br=0;
    
  }  
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", onTwist);

// Motor Control function
void Move_motor(double speed_val,const uint8_t pwm,const uint8_t forw,const uint8_t back)
{
  int speed_pwm = map(int(fabs(speed_val*100)), 0, int(0.3*100), 0, 255);
  if(speed_val >= 0.0)
  {
    digitalWrite(forw, HIGH);
    digitalWrite(back, LOW);
    analogWrite(pwm, abs(speed_pwm));
    nh.loginfo("Motor_run"); 
  }
  else if(speed_val < 0.0)
  {
    digitalWrite(forw, LOW);
    digitalWrite(back, HIGH);
    analogWrite(pwm, abs(speed_pwm));
    nh.loginfo("Motor_back");
  }

}

void setpins()
{
  pinMode(LF_FORW,OUTPUT);
  pinMode(LF_BACK,OUTPUT);
  pinMode(RF_FORW,OUTPUT);
  pinMode(RF_BACK,OUTPUT);
  pinMode(LF_PWM,OUTPUT);
  //pinMode(RF_PWM,OUTPUT);
  pinMode(LB_FORW,OUTPUT);
  pinMode(LB_BACK,OUTPUT);
  pinMode(RB_FORW,OUTPUT);
  pinMode(RB_BACK,OUTPUT);
  //pinMode(LB_PWM,OUTPUT);
  //pinMode(RB_PWM,OUTPUT);

  digitalWrite(RF_FORW, HIGH);
  digitalWrite(RF_BACK, LOW);
  digitalWrite(LF_FORW, HIGH);
  digitalWrite(LF_BACK, LOW);
  digitalWrite(RB_FORW, HIGH);
  digitalWrite(RB_BACK, LOW);
  digitalWrite(LB_FORW, HIGH);
  digitalWrite(LB_BACK, LOW);
}

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

void setup()
{
  nh.initNode();
  setpins();
  nh.subscribe(sub);
}

// maximum time that the system should wait before stopping the motor
// if there is no velocity command
int max_cmd_hold_time = 3000;

void loop()
{
  //wait until you are actually connected
  nh.spinOnce();

  now = millis();

  if (now - prev <= max_cmd_hold_time){
    // Move the motors with the output of the pid
    Move_motor(Output_fl,LF_PWM,LF_FORW,LF_BACK);
    Move_motor(Output_fr,RF_PWM,RF_FORW,RF_BACK);
    Move_motor(Output_bl,LB_PWM,LB_FORW,LB_BACK);
    Move_motor(Output_br,RB_PWM,RB_FORW,RB_BACK);
  }
  else{
    stop();
  }
  
  a = 0;
  delay(25);
}
