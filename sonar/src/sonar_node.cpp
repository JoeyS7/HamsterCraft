/*	
 sonar_node.cpp
	-Queries sonar microprocessor on I2C
	-Publishes result on sonar topic
	
*/

#include "ros/ros.h"
#include "common_files/ReadI2C.h"
#include "common_files/WriteI2C.h"
#include "common_files/Sonar.h"

int main(int argc, char **argv){
	ros::init(argc, argv, "sonar_node");
	ros::NodeHandle n;
	
	ros::Publisher i2c_write = n.advertise<common_files::WriteI2C>("write_i2c", 1000);

	ros::ServiceClient i2c_read = n.serviceClient<common_files::ReadI2C>("read_i2c");

	ros::Publisher sonar_topic = n.advertise<common_files::Sonar>("sonar_topic", 1000);
	
	while(ros::ok()){
		//write command byte to indicate sonar read
		common_files::WriteI2C command;
		command.addr = 1;
		command.data.push_back(1);
		i2c_write.publish(command);
	
		//read i2c
		common_files::ReadI2C sonar_srv;
		sonar_srv.request.addr = 1;

		if(i2c_read.call(sonar_srv)){
			//publish result
			common_files::Sonar sonar_msg;
			sonar_msg.sonar1 = sonar_srv.response.data[0];
			sonar_msg.sonar2 = sonar_srv.response.data[1];
			sonar_msg.sonar3 = sonar_srv.response.data[2];	
			sonar_topic.publish(sonar_msg);
		}else{
			ROS_ERROR("No sonar response received");
		}

		ros::Duration(.1).sleep(); //10 Hz sampling
		
	}
	
	return 0;	
}


