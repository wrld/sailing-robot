#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <pixhawk/imu_gps.h>
#include <std_msgs/Float32.h>
#include <sailing_robot/Velocity.h>
#include <cmath>
#include <sensor_msgs/NavSatFix.h>

serial::Serial ser;
int num;
int i;
float latitude;
float lontitude;
float altitude;
float vx;
float vy;
float vz;
float angle;
pixhawk::imu_gps imugps;
std_msgs::Float32 angle_float32;
sailing_robot::Velocity velocity;
sensor_msgs::NavSatFix position;
float time_last=0;
float time_now=0;
float time_inv=0;

void write_callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO_STREAM("Writing to serial port" << msg->data);
    ser.write(msg->data);
}

int main (int argc, char** argv){
    ros::init(argc, argv, "imu_gps_serial_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    ros::Publisher read_pub = nh.advertise<pixhawk::imu_gps>("imu_gps", 1000);
    ros::Publisher heading_pub = nh.advertise<std_msgs::Float32>("heading",10);
    ros::Publisher velocity_pub = nh.advertise<sailing_robot::Velocity>("gps_velocity",10);
    ros::Publisher pos_pub = nh.advertise<sensor_msgs::NavSatFix>("position",10);
    try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(57600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
	serial::bytesize_t bytes = serial::eightbits;
	serial::parity_t parity=serial::parity_none;
	serial::stopbits_t stopbits=serial::stopbits_one;
        ser.setBytesize(bytes);
        ser.setParity(parity);
        ser.setStopbits(stopbits);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(10);
    while(ros::ok()){

        ros::spinOnce();

        if(ser.available()){
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            num=ser.available();
            result.data = ser.read(num);
	    ser.flushInput();
	    //ROS_INFO_STREAM("num: " << num);
	    //for(i=0;i<num;i++)
		//{
		   //ROS_INFO_STREAM("raw_data: " <<(unsigned int)(unsigned char)(result.data[i]));
		//}
            if((unsigned int)(unsigned char)result.data[5]==33&&(unsigned int)(unsigned char)result.data[1]==28)
		{
                vx=((unsigned char)(result.data[26])+256.0*(unsigned char)(result.data[27]))/100.0f;
                vy=((unsigned char)(result.data[28])+256.0*(unsigned char)(result.data[29]))/100.0f;
                vz=((unsigned char)(result.data[30])+256.0*(unsigned char)(result.data[31]))/100.0f;
                angle=((unsigned char)(result.data[32])+256.0*(unsigned char)(result.data[33]))/100.0f;
		if(angle>180)
		{angle=angle-360;}

		if(vx>655.36f/2)
		{vx=vx-655.36f;}
		if(vy>655.36f/2)
		{vy=vy-655.36f;}
		if(vz>655.36f/2)
		{vz=vz-655.36f;}

		if(result.data[13]<127)
                {latitude=((unsigned char)(result.data[10])+256.0*(unsigned char)(result.data[11])+256.0*256.0*(unsigned char)(result.data[12])+256.0*256.0*256.0*(unsigned char)(result.data[13]))/10000000.0;}
		else
                {latitude=((unsigned char)(result.data[10])+256.0*(unsigned char)(result.data[11])+256.0*256.0*(unsigned char)(result.data[12])+256.0*256.0*256.0*(unsigned char)(result.data[13])-256.0*256.0*256.0*256.0)/10000000.0;}

		if(result.data[17]<127)
                {lontitude=((unsigned char)(result.data[14])+256.0*(unsigned char)(result.data[15])+256.0*256.0*(unsigned char)(result.data[16])+256.0*256.0*256.0*(unsigned char)(result.data[17]))/10000000.0;}
		else
                {lontitude=((unsigned char)(result.data[14])+256.0*(unsigned char)(result.data[15])+256.0*256.0*(unsigned char)(result.data[16])+256.0*256.0*256.0*(unsigned char)(result.data[17])-256.0*256.0*256.0*256.0)/10000000.0;}
		
		if(result.data[21]<127)
                {altitude=((unsigned char)(result.data[18])+256.0*(unsigned char)(result.data[19])+256.0*256.0*(unsigned char)(result.data[20])+256.0*256.0*256.0*(unsigned char)(result.data[21]))/1000.0;}
		else
                {altitude=((unsigned char)(result.data[18])+256.0*(unsigned char)(result.data[19])+256.0*256.0*(unsigned char)(result.data[20])+256.0*256.0*256.0*(unsigned char)(result.data[21])-256.0*256.0*256.0*256.0)/1000.0;}
	
		time_last=time_now;
                time_now=((unsigned char)(result.data[6])+256.0*(unsigned char)(result.data[7])+256.0*256.0*(unsigned char)(result.data[8])+256.0*256.0*256.0*(unsigned char)(result.data[9]))/1000.0;
		time_inv=time_now-time_last;
		if(time_inv>0)
		{}

		ROS_INFO_STREAM("latitude: "<<latitude);
		ROS_INFO_STREAM("lontitude: "<<lontitude);
		ROS_INFO_STREAM("altitude: "<<altitude);
		ROS_INFO_STREAM("vx: "<<vx);
		ROS_INFO_STREAM("vy: "<<vy);
		ROS_INFO_STREAM("vz: "<<vz);
		ROS_INFO_STREAM("angle: "<<angle);
		ROS_INFO_STREAM("time_inv: "<<time_inv);

		imugps.latitude=latitude;
		imugps.lontitude=lontitude;
		imugps.vx=vx;
		imugps.vy=vy;
		imugps.vz=vz;
		imugps.angle=angle;
		read_pub.publish(imugps);

		angle_float32.data=angle;
		heading_pub.publish(angle_float32);

		velocity.speed=sqrt(vx * vx + vy * vy);
		velocity.heading=atan2((double)vx,(double)vy)*180/3.14f;
		velocity_pub.publish(velocity);

		position.latitude=(double)latitude;
		position.longitude=(double)lontitude;
		position.altitude=(double)altitude;
		pos_pub.publish(position);
		}
        }
        loop_rate.sleep();
    }
}

