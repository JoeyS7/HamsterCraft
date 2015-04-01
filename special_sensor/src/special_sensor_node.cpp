#include "ros/ros.h"
#include "common_files/ReadI2C.h"
#include "common_files/WriteI2C.h"
//#include "common_files/Mice.h"

int main(int argc, char** argv){
	ros::init(argc, argv, "special_sensor_node");
	ros::NodeHandle n; //Create nodehandle object

	ros::Publisher write_i2c = n.advertise<common_files::WriteI2C>("write_i2c", 1000);
	ros::ServiceClient read_i2c = n.serviceClient<common_files::ReadI2C>("read_i2c");
//	ros::Publisher mice_topic = n.advertise<common_files::WriteI2C>("mice", 1000);
	
	while(ros::ok()){
		//write_i2c
		common_files::WriteI2C command;
		command.addr = 2;
		command.data.push_back(1);
		write_i2c.publish(command);

		common_files::ReadI2C sensor_srv;
		sensor_srv.request.addr = 2;

		if(read_i2c.call(sensor_srv)){
//			common_files::Mice;
			//TODO: COPY ReadI2C DATA OVER TO MICE
		}else{
			ROS_ERROR("No response from mice");
		}

		ros::spinOnce();
		ros::Duration(.1).sleep();

	}

	return 0;
}
	
	
	
	
	

