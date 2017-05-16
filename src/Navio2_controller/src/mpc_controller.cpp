	#include "Util.h"
	#include <unistd.h>
    #include <fstream>
	#include "ros/ros.h"
	#include "sensor_msgs/Imu.h"
	#include <sstream>

float timeState    = 0;
float oldTimeState = 0;

int freq = 5; //dt for controller is 0.2

// state vector (x, y, yaw, virtual state, roll, speed)
float currentState[6][1]    = {{0.0},{0.0},{0.0},{0.0},{0.0},{0.0}};
// output control (roll reference, speed reference)
float toSendControl[2][1]   = {{0.0}, {4.0}}


void read_State(sensor_msgs::Imu state_msg)
{
    //save the time of the aquisition
    oldtimeState = timeState;
    timeState = state_msg.header.stamp.toSec();
    
    //save current state
    currentState[0][0] = state_msg.orientation.x;       //x
    currentState[1][0] = state_msg.orientation.y;       //y
    currentState[2][0] = state_msg.orientation.z;       //yaw
    currentState[4][0] = state_msg.angular_velocity.x;  //roll
    currentState[5][0] = state_msg.angular_velocity.y;  //speed
}


	int main(int argc, char **argv)
	{

	 	/***********************/
		/* Initialize The Node */
		/***********************/
		ros::init(argc, argv, "MPC_controller");
		ros::NodeHandle n;
        
        //it publishes controls that will be read by the remote node
        ros::Publisher control_pub = n.advertise<sensor_msgs::Temperature>("mpc_control", 1000);
        
        //subscribe to state estimation topic
        ros::Subscriber state_sub = n.subscribe("estimated_state", 1000, read_State);
        
        
		//running rate = freq Hz
		ros::Rate loop_rate(freq);
        
        
        /***********************/
        /* Initialize Message */
        /***********************/
        sensor_msgs::Temperature control_msg;
        
        
        while (ros::ok()){
        
            //read last state as estimated by the Kalman filter ?? done automatically ??
            
            //run MPC algorithm
            
            //get controls to apply
            
            //propagate predicted state (for LPV linearization)
            //BEWARE OF CURVILINEAR ABSCISSA THAT HAS TO BE PROPAGATED TOO
            
            //send controls through messages
            //save values into msg container
            control_msg.header.stamp = ros::Time::now();
            control_msg.temperature = toSendControl[0][0]; // send roll reference
            control_msg.variance = toSendControl[1][0];    // send speed reference
            
            //publish message
            control_pub.publish(ctrl_msg);

            //looping
            ros::spinOnce();
            loop_rate.sleep();

		}


	  	return 0;
	}

