#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Encoder.h>


///////////////pins/////////////////////////////////////
#define Button1  24 
#define Button2  26 
#define Button3  28 
#define Button4  30 
#define Button5  30 //TODO popravi povezavo pin 33-32 
#define Button6  36
#define Button7  34
#define Button8  38

#define LMotPWMPin  45 
#define RMotPWMPin  46
#define LMotAPin 35
#define LMotBPin 39
#define RMotAPin 37
#define RMotBPin 41
int LMotor = 1;
int RMotor = 0;

#define IsenPin A8

#define GLEDPin 9
#define BLEDPin 8

#define LEncoderA 21
#define LEncoderB 20
#define REncoderA 2
#define REncoderB 3



//////////////////////////////////////////////////////////////////////////////////////

float WheelSeparation = 0.6;
float WheelDiameter = 0.2;
int TPR = 6400; //Encoder ticks per rotation 
int IMax = 300;
int AccParam = 3; //acceleration multiplier. 
//sudo vars//////////////////////////////////////////////////////////////////////////
int OdomWait = 3;
int OdomCount = 0;
double WCS[2] = {0,0};

//ROS variables//////////////////////////////////////////////////////////////////////
ros::NodeHandle nh;
////ROS publisher
geometry_msgs::Twist odom_msg;
ros::Publisher Pub ("ard_odom", &odom_msg);
geometry_msgs::Twist debug_msg;
ros::Publisher Debug ("debug", &debug_msg);
//ROS subscriber

void messageCb( const geometry_msgs::Twist& CVel){
	//geometry_msgs::Twist twist = twist_msg;   
    double vel_x = CVel.linear.x;
    double vel_th = CVel.angular.z;
    double right_vel = 0.0;
    double left_vel = 0.0;

    // turning
    if(vel_x == 0){  
        right_vel = vel_th * WheelSeparation / 2.0;
        left_vel = (-1) * right_vel;
    }
    // forward / backward
    else if(vel_th == 0){ 
        left_vel = right_vel = vel_x;
    }
    // moving doing arcs
    else{ 
        left_vel = vel_x - vel_th * WheelSeparation / 2.0;
        right_vel = vel_x + vel_th * WheelSeparation / 2.0;
    }
    //write new command speeds to global vars 
    WCS[0] = left_vel;
    WCS[1] = right_vel;
}

ros::Subscriber<geometry_msgs::Twist> Sub("cmd_vel", &messageCb );

/////////////////////////////////////////////////////////////////////////////////////
int Button[] = {Button1, Button2, Button3, Button4, Button5, Button6, Button7, Button8};
int ButtonPress[] = {0,0,0,0,0,0,0,0}; 


//Motor vars/////////////////////////////////////////////////////////////////////////
#define CW   1
#define CCW  2
int inApin[2] = {LMotAPin, RMotAPin};  // INA: Clockwise input
int inBpin[2] = {LMotBPin, RMotBPin}; // INB: Counter-clockwise input
int pwmpin[2] = {LMotPWMPin, RMotPWMPin}; // PWM input
int MotorNum[2] = {LMotor, RMotor};

//encoder vars
Encoder LEncoder(LEncoderA, LEncoderB);
Encoder REncoder(REncoderA, REncoderB);

long EncoderVal[2] = {0,0};
double DDis[2] = {0,0};
long Time[2] = {0,0};

//debug
double Vels[2] = {0,0};
int CVEL[2]= {0,0};
int Mspeeds[2] = {0,0};

///program///////////////////////////////////////////////////////////////////////////
void setup()
{
	nh.getHardware()->setBaud(115200);
    nh.initNode();	
    nh.advertise(Pub);
    nh.advertise(Debug);
    nh.subscribe(Sub);

    nh.getParam("/serial_node/WheelSeparation", &WheelSeparation,1);
    nh.getParam("/serial_node/WheelDiameter", &WheelDiameter,1);
    nh.getParam("/serial_node/IMax", &IMax,1);
    nh.getParam("/serial_node/AccParam", &AccParam,1);


}

void loop(){

    nh.spinOnce();
    
    //first couple of times dont publish odom
    if(OdomCount > OdomWait){
	    odom_msg.linear.x = Vels[0];
	    odom_msg.linear.y = Vels[1];
		Pub.publish(&odom_msg);
	}
	else{OdomCount++;}

    debug_msg.linear.x = WCS[0];
    debug_msg.linear.y = Vels[0];
    debug_msg.linear.z = Mspeeds[0];
    debug_msg.angular.x = WCS[1];
    debug_msg.angular.y = Vels[1];
    debug_msg.angular.z= Mspeeds[1];


    Debug.publish(&debug_msg);

    //safeswitch
    if(CheckBumpers()){
    	WCS[0]=0;
    	WCS[1]=0;
    }

    MotorWrite();	//Takes WCS and corrects speed of motors with encoders    

    delay(3);
}


int CheckBumpers(){
	int ToReturn = 0;
	for(int i = 0;i<8;i++){
		if(digitalRead(Button[i])) ToReturn = 1;
	}
	return ToReturn;
}

//encoder code//////////////////////////////////////////////////////////

//motor write speed - in motor units
double MWS[2]= {0,0};

double CorrectedSpeed(int M, double CVel){
	//if fist time in program return 0 and init time vars
	if(Time[0]==0 && Time[1] == 0){
		Time[0] = millis();
		Time[1] = millis();
		return 0;
	}

	//read encoder ticks
	if(M == LMotor){
		EncoderVal[0] = LEncoder.read();
		LEncoder.write(0);
	}
	if(M == RMotor){
		EncoderVal[1] = REncoder.read();
		REncoder.write(0);
	}

	//differencial of time in seconds
	long T = millis();
	int DTime = T-Time[M];
	Time[M] = T;


	//diferential of distance in meters
	DDis[M] = TicksToMeters(EncoderVal[M]);
	
	//calculate short term measured velocity
	double EVel = (DDis[M]/DTime)*1000;
	
	//save to publish to /ard_odom
	Vels[M] = EVel;

	EVel = abs(EVel);
	CVel = abs(CVel);

	double dif = EVel - CVel;

	if(MWS[M]<60 && MWS[M]>=0){MWS[M]=MWS[M]-(dif*AccParam);}
	if(MWS[M]>60){MWS[M]=59;}
	if(MWS[M]<0){MWS[M]=0;}
	
	if(CVel == 0){MWS[M] = 0;}

	//DEBUG
	CVEL[M] = MWS[M];

	return MWS[M];

}

double TicksToMeters(int Ticks){
	return (Ticks*3.14*WheelDiameter)/TPR;
}

//motor codes///////////////////////////////////////////////////////////
void MotorWrite(){
	int DIR;

	for(int i = 0; i<2; i++){
		//correct turns of motors
		DIR = CW;
		if((WCS[i]>0&&i==MotorNum[0])||(WCS[i]<0&&i==MotorNum[1])){DIR=CCW;}

		//correct speed with encoder data
		double MSpeed = CorrectedSpeed(i, WCS[i]);
		
		//if current is too high, stop robot
		if(analogRead(IsenPin)>IMax) MSpeed = 0;
		
		Mspeeds[i]=MSpeed;

		//debug
		//MSpeed = abs(WCS[i]);
		//if(MSpeed>30) MSpeed = 0;
		
		motorGo(MotorNum[i], DIR, int(MSpeed));
						
				
	}
}


void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1){
    if (direct <=4){
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}