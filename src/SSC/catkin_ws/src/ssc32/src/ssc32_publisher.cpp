#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include "std_msgs/Int32MultiArray.h"

int main(int argc, char **argv)
{
    
	ros::init(argc, argv, "ssc32_publisher");

	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<std_msgs::Int32MultiArray>("ssc32/angles", 100);

	std_msgs::Int32MultiArray array;

	for (int i = 0; i < 6; ++i)
	{
		array.data.push_back(1500) ;
	}

	ros::spinOnce();
	ROS_INFO("I published something!\n");

	int initial = 1500;
	int count = 0;

	while (ros::ok())
	{
		if (count % 2 == 0)
			initial += 500;
		else
			initial -= 500;
		//Clear array
		array.data.clear();
		for (int i = 0; i < 6; ++i)
		{
			array.data.push_back(initial);
		}
		//Publish array
		pub.publish(array);
		//Let the world know
		ROS_INFO("I published something!\n");
		//Do this.
		ros::spinOnce();
		//Added a delay so not to spam
		sleep(2);
		count++;
	}

}

//Structure of the array : 
// {angle0, angle1, angle2, angle3, angle4, angle5}