#include "MPU9250.h"
#include "LSM9DS1.h"
#include "Util.h"
#include "AHRS.hpp"
#include <unistd.h>
#include <sys/time.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include <sstream>
#include <float.h>

#define G_SI 9.80665
#define PI   3.14159

//Found after magnetometer calibration
//#define MAG_OFFSETX 38.6876
//#define MAG_OFFSETY 34.0460
//#define MAG_OFFSETZ -25.0617
//#define MAG_SCALEX 0.0069
//#define MAG_SCALEY 0.0098
//#define MAG_SCALEZ 0.0126

//#define EARTH_MAG_FIELD 0.47  //(Gauss) in Switzerland

//#define clear() printf("\033[H\033[J")
// Objects

InertialSensor *imu;
AHRS    ahrs;   // Mahony AHRS

// Sensor data

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

// Orientation data

float roll, pitch, yaw;

// Timing data

//Gyro offset
float offset[3];

//Magneto calibration
float mag_bias[3];
float mag_scale[3];


struct timeval tv;
float dt, maxdt;
float mindt = 0.01;
unsigned long previoustime, currenttime;
float dtsumm = 0;
int isFirst = 1;

int freq = 100;

//============================= Initial setup =================================

void imuSetup()
{
    //----------------------- MPU initialization ------------------------------

    imu->initialize();

    //-------------------------------------------------------------------------

    
	printf("Beginning Gyro calibration...\n");
	for(int i = 0; i<100; i++)
	{
		imu->update();
    		imu->read_gyroscope(&gx, &gy, &gz);

		gx *= 180 / PI;
		gy *= 180 / PI;
		gz *= 180 / PI;

		offset[0] += (-gx*0.0175);
		offset[1] += (-gy*0.0175);
		offset[2] += (-gz*0.0175);
		usleep(10000);
	}
	offset[0]/=100.0;
	offset[1]/=100.0;
	offset[2]/=100.0;

	printf("Offsets for gyro are: %f %f %f\n", offset[0], offset[1], offset[2]);
	ahrs.setGyroOffset(offset[0], offset[1], offset[2]);
    
    
    printf("Beginning Magneto calibration...\n");
    
    
    uint16_t ii = 0, sample_count = 0;
    
    float mag_max[3]={-FLT_MAX, -FLT_MAX, -FLT_MAX};
    float mag_min[3]={FLT_MAX, FLT_MAX, FLT_MAX};
    
    printf("%f, %f\n", mag_max[0], mag_min[0]);
    sleep(2);

    printf("Move Motorcycle until done!\n");
    sleep(4);
    
    sample_count = 200;
    for(ii = 0; ii < sample_count; ii++) {
	printf("Getting new sample\n");
	imu->update();
        imu->read_magnetometer(&mx, &my, &mz);
        if(mx > mag_max[0]) mag_max[0] = mx;
        if(mx < mag_min[0]) mag_min[0] = mx;
        if(my > mag_max[1]) mag_max[1] = my;
        if(my < mag_min[1]) mag_min[1] = my;
        if(mz > mag_max[2]) mag_max[2] = mz;
        if(mz < mag_min[2]) mag_min[2] = mz;
        usleep(100000); //Let time for the magneto to change values
    }
    
    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts
    
    printf("Mag Calibration done!\n");
    printf("Offsets for magneto are: %f %f %f\n", mag_bias[0], mag_bias[1], mag_bias[2]);

    sleep(6);
    
}

//============================== Main loop ====================================

void imuLoop()
{

    float dtsum = 0.0f; //sum of delta t's

    while(dtsum < 1.0f/freq) //run this loop at 1300 Hz (Max frequency gives best results for Mahony filter)
    {
		//----------------------- Calculate delta time ----------------------------

		gettimeofday(&tv,NULL);
		previoustime = currenttime;
		currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
		dt = (currenttime - previoustime) / 1000000.0;
		if(dt < 1/1300.0) usleep((1/1300.0-dt)*1000000);
		gettimeofday(&tv,NULL);
		currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
		dt = (currenttime - previoustime) / 1000000.0;

	    //-------- Read raw measurements from the MPU and update AHRS --------------

	    // Accel + gyro.
	
	    imu->update();
	    imu->read_accelerometer(&ax, &ay, &az);
	    imu->read_gyroscope(&gx, &gy, &gz);

	    ax /= G_SI;
	    ay /= G_SI;
	    az /= G_SI;
	    gx *= 180 / PI;
	    gy *= 180 / PI;
	    gz *= 180 / PI;

	    //ahrs.updateIMU(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, dt);
	    	
	    // Accel + gyro + mag.
	    // Soft and hard iron calibration required for proper function.
	    
	   /* imu->update();
	    imu->read_accelerometer(&ay, &ax, &az);
		az *= -1;
	    imu->read_gyroscope(&gy, &gx, &gz);
		gz *= -1;*/
	    imu->read_magnetometer(&mx, &my, &mz);
        
        mx = (mx - mag_bias[0])/mag_scale[0];
        my = (my - mag_bias[1])/mag_scale[1];
        mz = (mz - mag_bias[2])/mag_scale[2];
	printf("yaw magneto %f, %f, %f, %f, %f\n", atan2(mx,my)*180/3.14, atan2(my, mx)*180/3.14, mx, my, mz);
	   /* ax /= G_SI;
	    ay /= G_SI;
	    az /= G_SI;
	    gx *= 180 / PI;
	    gy *= 180 / PI;
	    gz *= 180 / PI;*/

	   ahrs.update(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, mx, my, -mz, dt);
	    

	    //------------------------ Read Euler angles ------------------------------

	    ahrs.getEuler(&pitch, &roll, &yaw);

	    //------------------- Discard the time of the first cycle -----------------

	    if (!isFirst)
	    {
	    	if (dt > maxdt) maxdt = dt;
	    	if (dt < mindt) mindt = dt;
	    }
	    isFirst = 0;
	
	dtsum += dt;
    }


    
}

void init_imu_msg(sensor_msgs::Imu* imu_msg)
{
	//time stamp
	imu_msg->header.stamp = ros::Time::now();
	
	imu_msg->orientation.x = 0.0f;
	imu_msg->orientation.y = 0.0f;
	imu_msg->orientation.z = 0.0f;
	imu_msg->orientation.w = 0.0f;

	imu_msg->angular_velocity.x = 0.0f;
	imu_msg->angular_velocity.y = 0.0f;
	imu_msg->angular_velocity.z = 0.0f;

	imu_msg->linear_acceleration.x = 0.0f;
	imu_msg->linear_acceleration.y = 0.0f;
	imu_msg->linear_acceleration.z = 0.0f;

	for(int i = 0; i < 9; i++)
	{
		imu_msg->orientation_covariance[i] = !(i%4);
		imu_msg->angular_velocity_covariance[i] = !(i%4);
		imu_msg->linear_acceleration_covariance[i] = !(i%4);
	}
}

void init_mf_msg(sensor_msgs::MagneticField* mf_msg)
{
	//time stamp
	mf_msg->header.stamp = ros::Time::now();
	
	mf_msg->magnetic_field.x = 0.0f;
	mf_msg->magnetic_field.y = 0.0f;
	mf_msg->magnetic_field.z = 0.0f;

	for(int i = 0; i < 9; i++)
	{
		mf_msg->magnetic_field_covariance[i] = !(i%4);
	}
}

void update_imu_msg(sensor_msgs::Imu* imu_msg, InertialSensor* imu)
{
	//time stamp
	imu_msg->header.stamp = ros::Time::now();
	
	//float ax, ay, az, gx, gy, gz;

	//imu->update();

	//imu->read_accelerometer(&ax, &ay, &az);
        //imu->read_gyroscope(&gx, &gy, &gz);

	imu_msg->orientation.x = roll;
	imu_msg->orientation.y = pitch;
	imu_msg->orientation.z = yaw;
	imu_msg->orientation.w = dt;

	imu_msg->angular_velocity.x = gx;
	imu_msg->angular_velocity.y = gy;
	imu_msg->angular_velocity.z = gz;

	imu_msg->linear_acceleration.x = ax*G_SI;
	imu_msg->linear_acceleration.y = ay*G_SI;
	imu_msg->linear_acceleration.z = az*G_SI;

	printf("Attitude: [Roll:%+05.2f]  [Pitch:%+05.2f]  [Yaw:%+05.2f] \n [Period:%.4fs]  [Rate:%dHz] \n \n", roll, pitch, yaw, dt, int(1/dt));

}

void update_mf_msg(sensor_msgs::MagneticField* mf_msg, InertialSensor* imu)
{
	//time stamp
	mf_msg->header.stamp = ros::Time::now();
	
	//float mx, my, mz;

	//imu->update();

        //imu->read_magnetometer(&mx, &my, &mz);

	mf_msg->magnetic_field.x = mx;
	mf_msg->magnetic_field.y = my;
	mf_msg->magnetic_field.z = mz;

	//ROS_INFO("Magnetic Field : X = %+7.3f, Y = %+7.3f, Z = %+7.3f", mx, my, mz);
}

int main(int argc, char **argv)
{
	// The parameter to this function is the running frequency
	if(argc == 2)
	{
		if(atoi(argv[1]) > 0)
		{
			freq = atoi(argv[1]);
			printf("Frequency selected : %d \n", freq);
		}
		else
		{
			printf("Frequency must be more than 0");
			return 0;
		}
	}
		


 	/***********************/
	/* Initialize The Node */
	/***********************/
	ros::init(argc, argv, "imu_mpu9250_handler");
	ros::NodeHandle n;
	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu_readings", 1000);
	ros::Publisher mf_pub = n.advertise<sensor_msgs::MagneticField>("mag_readings", 1000);

	ros::Rate loop_rate(freq);

	/*************************/
	/* Initialize the Sensor */
	/*************************/

	printf("Selected: MPU9250\n");
	imu = new MPU9250();

	/***************/
	/* Test Sensor */
	/***************/
	if (!imu->probe()) 
	{
		printf("Sensor not enabled\n");
		return EXIT_FAILURE;
	}

	imuSetup();

	while (ros::ok())
	{
		//acquire data
		imuLoop();

		//create messages
		sensor_msgs::Imu imu_msg;
		sensor_msgs::MagneticField mf_msg;
		
		//initialize messages with 0's
		init_imu_msg(&imu_msg);
		init_mf_msg(&mf_msg);

		//put real measured values in the messages
		update_imu_msg(&imu_msg, imu);
		update_mf_msg(&mf_msg, imu);

		//publish the messages on the topics
		imu_pub.publish(imu_msg);
		mf_pub.publish(mf_msg);

		ros::spinOnce();

		loop_rate.sleep();

	}


  	return 0;
}
