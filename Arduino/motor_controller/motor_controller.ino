//ros based UV sanitation robot 
//pid based drive control
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

//constants
#define SECS  60 // seconds
#define RPR  7  // encoder pulses per rotation
#define GEARRATIO  270
#define WRADIUS  0.05
#define WBASE  0.48
#define RCONST  0.104719755 // rpm to rad/s conversion constant
#define DEADLOCK 5 //motors are disengaged when no communication occurs for X seconds
//motor A
const int encoderPinA1 = 2;
const int encoderPinA2 = 8;
const int E1 =9;
const int I1 = 10;
const int I2 = 11;

//motor B
int encoderPinB1 = 3;
int encoderPinB2 = 4;
int E2 =5;
int I3 = 6;
int I4 =7;

static uint32_t secsA = 0; //time between successive revolutions
static uint32_t dirA = 0; 

double rpmA;  //motor rpm
double lrpmA = 0; //prevrpm
double vA;  // in m/min

static uint32_t secsB = 0; //time between successive revolutions
static uint32_t dirB = 0; 


double rpmB;  //motor rpm
double lrpmB; //prevrpm
double vB;   // in m/min


double speed_req_A =0;
double speed_req_B =0;
double pwmA = 0;
double pwmB = 0;

int lastMilli = 0;
int   deadloops = 0; 

double KpA = 7.5, KiA = 14 ,KdA = 0.14;
double KpB = 7.5, KiB=13 ,KdB = 0.5;

PID PIDA(&vA, &pwmA, &speed_req_A, KpA, KiA, KdA, DIRECT);
PID PIDB(&vB, &pwmB, &speed_req_B, KpB, KiB, KdB, DIRECT);

void handle_cmd (const geometry_msgs::Twist& cmd_vel);
void drive(int speedA,int speedB);

ros::NodeHandle nh;
geometry_msgs::Vector3Stamped speed_msg; 

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);                                
ros::Publisher speed_pub("speed", &speed_msg); 

void drive(double speed_req_A,double speed_req_B){
  if (speed_req_A < 0)
  {
    digitalWrite(I1,HIGH);
    digitalWrite(I2,LOW);
    dirA = -1;  

  }
  else
  if (speed_req_A > 0)
  {
    digitalWrite(I1,LOW);
    digitalWrite(I2,HIGH);
    dirA = 1;
  }
  else
  {
    digitalWrite(I1,LOW);
    digitalWrite(I2,LOW);  
    dirA =0;

  }
  if (speed_req_B < 0)
  {
    digitalWrite(I3,HIGH);
    digitalWrite(I4,LOW);
    dirB = -1;
  
  }
  else
  if (speed_req_B > 0)
  {
    digitalWrite(I3,LOW);
    digitalWrite(I4,HIGH); 
    dirB = 1; 
  }
  else
  {
    digitalWrite(I3,LOW);
    digitalWrite(I4,LOW);
    dirB =0;    

  } 

}

void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  deadloops = 0; 
                                                   
  int speed_req = cmd_vel.linear.x  * 60;                                    
  int angular_speed_req = cmd_vel.angular.z * 60;   
  
  speed_req_A = speed_req - angular_speed_req*(WBASE/2);
  speed_req_B = speed_req + angular_speed_req*(WBASE/2); 
    
  drive(speed_req_A,speed_req_B);
  speed_req_A = abs(speed_req_A);
  speed_req_B = abs(speed_req_B);
}

void publishSpeed(int looptime) {
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = dirA * vA /60;    //left wheel speed (in m/s)
  speed_msg.vector.y = dirB * vB /60;   //right wheel speed (in m/s)
  speed_msg.vector.z = looptime; 
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry\n");
}

void updateEncoderA(){
static uint32_t tempA;
  secsA = micros() - tempA;
  tempA = micros();
}

void updateEncoderB(){
static uint32_t tempB;
  secsB = micros() - tempB;
  tempB = micros();
}

void setup()
{
  nh.initNode();                       
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd_vel);                  
  nh.advertise(speed_pub); 
  
  pinMode(encoderPinA1, INPUT); 
  pinMode(encoderPinA2, INPUT);
  pinMode(encoderPinB1, INPUT); 
  pinMode(encoderPinB2, INPUT);

  digitalWrite(encoderPinA1, HIGH);
  digitalWrite(encoderPinA2, HIGH); 
  digitalWrite(encoderPinB1, HIGH);
  digitalWrite(encoderPinB2, HIGH);
  
  pinMode(E1, OUTPUT);
  pinMode(I1, OUTPUT); 
  pinMode(I2, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(I3, OUTPUT); 
  pinMode(I4, OUTPUT);  

  digitalWrite(E1,LOW);
  digitalWrite(E2,LOW);
  digitalWrite(I1,LOW);
  digitalWrite(I2,LOW);
  digitalWrite(I3,LOW);
  digitalWrite(I4,LOW);  
  
  attachInterrupt(0, updateEncoderA, RISING);
  attachInterrupt(1, updateEncoderB, RISING); 
  
  PIDA.SetMode(AUTOMATIC);
  PIDB.SetMode(AUTOMATIC);
}

void loop(){ 
  nh.spinOnce();
  if (!nh.connected())
  nh.loginfo("not connected\n");
  
  rpmA =  ((secsA == 0)?0: 1000000/secsA) *(SECS/RPR) /GEARRATIO;
  vA = rpmA * RCONST * WRADIUS * SECS;

  rpmB = ((secsB == 0)?0: 1000000/secsB) *(SECS/RPR) /GEARRATIO;
  vB = rpmB * RCONST * WRADIUS * SECS;
  
  PIDA.Compute();
  PIDB.Compute();
  analogWrite(E1,pwmA);
  analogWrite(E2,pwmB);
  
  if(lrpmB == rpmB)
    secsB = 0;
  lrpmB = rpmB;
  if (rpmA == lrpmA)
    secsA = 0;
  lrpmA = rpmA;
  delay(50);
  
  if((millis()-lastMilli) >= 250)   
  { 
    publishSpeed((millis()-lastMilli));  
    lastMilli = millis();
  }

  if (deadloops >= DEADLOCK * 20)
  drive(0.0,0.0);
  else
  deadloops++;
}
