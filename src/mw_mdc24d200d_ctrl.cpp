#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>


///////////////////////////////////////////////////////////////////////////	var. declaration
#define PI 3.141592

serial::Serial ser;									// serial object

bool d_out=false;									// digital output value
bool d_out_transmit_flag=false;						// digital output transmit flag

float vel_l=0.0;									// velocity of each motors
float vel_r=0.0;

char *cmd_vel;										// velocity commands
char cmd[5];

char cmd_l[5];
char cmd_r[5];

int n_digit=0;										// number digit, sign
int sign=0;


///////////////////////////////////////////////////////////////////////////	parameters
char mot1_max_vel[]="xv1=6000\r\n";					// setup commands
char mot2_max_vel[]="xv2=6000\r\n";
char mot1_max_acc[]="ac1=60000\r\n";
char mot2_max_acc[]="ac2=60000\r\n";
char mot1_max_dcc[]="dc1=60000\r\n";
char mot2_max_dcc[]="dc2=60000\r\n";
char mot1_max_vol[]="xvt1=12\r\n";
char mot2_max_vol[]="xvt2=12\r\n";
char mot1_enc_pulse[]="ep1=52\r\n";
char mot2_enc_pulse[]="ep2=52\r\n";
char mot1_vp_gain[]="vp1=0.15\r\n";
char mot2_vp_gain[]="vp2=0.15\r\n";
char mot1_vi_gain[]="vi1=0.001\r\n";
char mot2_vi_gain[]="vi2=0.001\r\n";
char mot1_cp_gain[]="cp1=0\r\n";
char mot2_cp_gain[]="cp2=0\r\n";
char mot1_ci_gain[]="ci1=50\r\n";
char mot2_ci_gain[]="ci2=50\r\n";
char sig_out_en[]="doe=1\r\n";
char sig_out_on[]="dov1=0\r\n";
char sig_out_off[]="dov1=1\r\n";
char mot1_vel[]="v1\r\n";
char mot2_vel[]="v2\r\n";

char mot_stop_cmd[]="mvc=0,0\r\n";					// motor velocity command
char mot_vel_cmd[]="mvc=00000,00000\r\n";			// motor velocity command

float tread=0.149;									// robot tread
float radius=0.0411*0.5;							// wheel radius
float g_ratio=170;									// wheel motor gear ratio

float max_vel=6000.0;								// max velocity


///////////////////////////////////////////////////////////////////////////	sub, pub class
class sub_pub						
{
private:
	ros::NodeHandle nh;
	ros::Subscriber d_out_sub;
    ros::Subscriber vel_l_sub;
    ros::Subscriber vel_r_sub;
    ros::Subscriber vel_sub;

public:
	sub_pub()										// subscriber, publisher declaration
	{
		d_out_sub=nh.subscribe("d_out", 1000, &sub_pub::d_out_callback, this);
		vel_sub=nh.subscribe("cmd_vel", 1000, &sub_pub::vel_callback, this);
	}
	
	void d_out_callback(const std_msgs::Bool::ConstPtr& msg)	// digital out call back func.
	{
		d_out_transmit_flag=true;
		d_out=msg->data;
	}
	
	void vel_callback(const geometry_msgs::Twist::ConstPtr& cmd)	// velocity call back func.
	{
		float lin_v=cmd->linear.x;
		float ang_v=cmd->angular.z;
		
		vel_l=(lin_v-tread*0.5*ang_v)/PI/radius*30.0*g_ratio;
		vel_r=(lin_v+tread*0.5*ang_v)/PI/radius*30.0*g_ratio;
		
		if(vel_l>max_vel)							// velocity limitation
			vel_l=6000.0;
		else if(vel_l<-max_vel)
			vel_l=-6000.0;
		
		if(vel_r>max_vel)
			vel_r=6000.0;
		else if(vel_r<-max_vel)
			vel_r=-6000.0;
	}
};


///////////////////////////////////////////////////////////////////////////	serial receive func.
void receive_data()
{
	std_msgs::String result;

	while(!ser.available());
	
	result.data=ser.read(ser.available());
	ROS_INFO_STREAM("Read: "<<result.data);
}


///////////////////////////////////////////////////////////////////////////	driver setup func.
void setup_driver()
{	
	ROS_INFO_STREAM("setting motor driver");
	
	ser.write(mot1_max_vel);
	receive_data();					
	ser.write(mot2_max_vel);
	receive_data();
	ser.write(mot1_max_acc);
	receive_data();					
	ser.write(mot2_max_acc);
	receive_data();
	ser.write(mot1_max_dcc);
	receive_data();					
	ser.write(mot2_max_dcc);
	receive_data();
	ser.write(mot1_max_vol);
	receive_data();
	ser.write(mot2_max_vol);
	receive_data();
	ser.write(mot1_enc_pulse);
	receive_data();
  	ser.write(mot2_enc_pulse);
  	receive_data();
  	ser.write(mot1_vp_gain);
  	receive_data();
  	ser.write(mot2_vp_gain);
  	receive_data();
	ser.write(mot1_vi_gain);
	receive_data();
	ser.write(mot2_vi_gain);
	receive_data();
	ser.write(mot1_cp_gain);
  	receive_data();
  	ser.write(mot2_cp_gain);
  	receive_data();
	ser.write(mot1_ci_gain);
	receive_data();
	ser.write(mot2_ci_gain);
	receive_data();
	ser.write(sig_out_en);
	receive_data();
	ser.write(sig_out_off);
	receive_data();
	
	ROS_INFO_STREAM("settings done");
}


///////////////////////////////////////////////////////////////////////////	int to char func.
char *itoa(char *str, int num)
{
	char *rev_buf;									// reversed character buffer
	char buf[5];
	
	n_digit=0;										// initialize number digit
	sign=0;											// sign
	rev_buf=buf;
	
	if(num!=0)						
	{
		if(num<0)									// find number sign
		{
			num=~num+1;
			sign=1;
		}
		
		for(n_digit=0; n_digit<5; n_digit++)		// generate reversed buffer
		{
			if(num<=0)								// find number digit
			{
				break;
			}
			
			rev_buf[n_digit]=(num%10)+'0';
			num/=10;			
		}
						
		for(int i=0; i<n_digit+sign; i++)
		{
			str[i]=rev_buf[n_digit-i-1];
		}
	}
	else											// return 0
	{
		n_digit=1;
		sign=0;
		str[0] ='0';
	}
	
	return str;
}


///////////////////////////////////////////////////////////////////////////	serial transmit func.
void transmit_vel(int v_l, int v_r)
{		
	for(int i=0; i<5; i++)							// initialize velocity string
	{
		cmd_l[i]='0';
		cmd_r[i]='0';
		cmd[i]='0';
	}
	
	cmd_vel=cmd;
	
	cmd_vel=itoa(cmd_vel, v_l);						// convert left int to char
	
	for(int i=0; i<n_digit; i++)
	{
		cmd_l[i+(5-n_digit)]=cmd_vel[i];
	}
	
	if(sign==1)
	{
		cmd_l[0]='-';
	}
	
	cmd_vel=itoa(cmd_vel, v_r);						// convert right int to char
	
	for(int i=0; i<n_digit; i++)
	{
		cmd_r[i+(5-n_digit)]=cmd_vel[i];
	}
	
	if(sign==1)
	{
		cmd_r[0]='-';
	}
	
	for(int i=0; i<5; i++)							// substitute velocity string to cmd
	{
		mot_vel_cmd[i+4]=cmd_l[i];
		mot_vel_cmd[i+10]=cmd_r[i];
	}
	
	ROS_INFO("%s", mot_vel_cmd);
	
	ser.write(mot_vel_cmd);							// transmit command
	receive_data();
}


///////////////////////////////////////////////////////////////////////////	main function
int main (int argc, char** argv)
{
	ros::init(argc, argv, "mw_mdc24d200d_ctrl");
	sub_pub sp;										// declare sub, pub class

	ros::Rate loop_rate(10);
	
	try												// setup serial communication
	{
    	ser.setPort("/dev/ttyUSB0");
    	ser.setBaudrate(115200);
    	serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    	ser.setTimeout(to);
    	ser.open();
	}
	catch (serial::IOException& e)
	{
    	ROS_ERROR_STREAM("Unable to open port ");
    	return -1;
	}

	if(ser.isOpen())
	{
    	ROS_INFO_STREAM("Serial Port initialized");
	}
	else
	{
    	return -1;
	}
	
	setup_driver();									// setup motor driver
	
	while(ros::ok())
	{
		if(d_out_transmit_flag==true)				// digital output value transmit
		{
			(d_out) ? ser.write(sig_out_on) : ser.write(sig_out_off);
			receive_data();
			
			d_out_transmit_flag=false;
		}
		
		transmit_vel((int)vel_l, (int)vel_r);					// velocity command transmit
    		
    	ros::spinOnce();        
    	loop_rate.sleep();
	}
	
	return 0;
}

