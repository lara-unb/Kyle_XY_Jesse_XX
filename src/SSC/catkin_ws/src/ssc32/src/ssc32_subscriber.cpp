/* Universidade de Brasília
 * Laboratório de Automação e Robótica
 * Autor: Gabriel Guimarães Almeida de Castro
 * Programa: Básico da comunicação serial e movimentação dos servos
 */

#include <iostream>
#include <string>
#include <cstdlib>
#include "ssc/ssc32.h"

//ROS
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include <vector>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

int Arr0[6];
int Arr1 = {750, 750, 750, 750, 750, 750};

void anglesCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{

	int i = 0;
	// print all the remaining numbers
	for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr0[i] = *it;
		i++;
	}

	return;
 	ROS_INFO("I am here\n");
}

int main(int argc, char **argv)
{
	ssc::SSC32 ssc32_device;
	//int ssc32_device.openPort();

	//ssc32_device.moveServo(0, 2500);
	ros::init(argc, argv, "ssc32_subscriber");
	
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("ssc32/angles", 100, anglesCallback);


	while(ros::ok())
	{
		ssc32_device.setPosSpeed(Arr0, Arr1);
		ssc32_device.moveServo();
		

		printf("\n");

		ros::spinOnce();
		sleep(2);
	}

	return 0;
}