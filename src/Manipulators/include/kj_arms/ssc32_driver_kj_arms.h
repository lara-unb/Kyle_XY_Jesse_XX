/************************************************************************************************************
*** Arquivo: ssc32_driver_kj_arms.h
*** Conteudo: SSC32 Driver Class
*** Modificado por: Felipe Luis Rodrigues Sousa
*** Código fonte: https://github.com/mnovo/lynxmotion_ros/blob/master/include/lynxmotion_ssc32/ssc32_driver.h
*************************************************************************************************************/

#ifndef LYNXMOTION_SSC32_SSC32_NODE_H
#define LYNXMOTION_SSC32_SSC32_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/JoinState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_srvs/Empty.h>
#include <XmlRcpValue.h>
#include <string>
#include <vector>
#include <queue>
#include "ssc32_kj_arms.h"

namespace lynxmotion_ssc32
{

struct Joint
{
	struct JointProperties
	{
		int channel;
		double min_angle;
		double max_angle;
		double offset_angle;	// Esse angulo eh considerado como sendo 1500 us
		double default_angle;	// Angulo que a juncao eh inicializada para (padrao para o offset_angle)
		bool initialize;		// Indica se deve iniciar o servo para o angulo padrao na inicializacao
		bool invert;
	};

	std::string name;
	JointProperties properties;
};

namespace ControllerTypes
{
	enum ControllerType
	{
		JointController,
		DiffDriveController
	};
}
typedef ControllerTypes::ControllerType ControllerType;

class SSC32Driver;

struct Controller
{
	std::string name;
	ControllerType Type;
	std::vector<Joint*> joints;	// Ponteiro para as juntas neste controlador
	bool publish_joint_states;
	double publish_rate;

private:
	double expected_publish_time;
	ros::Time last_publish_time;
	friend class SSC32Driver;
};

struct Command
{
	SSC32::ServoCommand *cmd;
	int num_joints;
	ros::Time start_time;
	ros::Duration duration;
};

class SSC32Driver
{
public:
	SSC32Driver(ros::NodeHandle &nh);
	~SSC32Driver();
	bool init();
	bool relaxJoints();
	bool relaxJointsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
	bool spin();
	bool start();
	void stop();
	void update();

private:
	void publishJointStates();
	void jointCallback(const ros::MessageEvent<trajectory_msgs::JointTrajectory const>& event);
	void execute_command(std::string);

	ros::NodeHandle nh;
	ros::ServiceServer relax_joints_service;
	std::vector<ros::Subscriber> joint_subs;
	std::map<std::string, ros::Publisher> joint_state_pubs_map;

	Joint *channels[6] // Lembrando que era definido por 32

	std::string port;
	int baud;
	bool publish_joint_states;
	double range_scale;
	double scale;
	std::vector<Controller*> controllers;
	std::map<std::string, Controller*> controllers_map;
	std::map<std::string, Joint*> joints_map;

	SSC32 ssc32_dev;

	ros::Time current_time;
	ros::Time last_time;

	std::map<std::string, std::queue<Command> > command_queues;

	// Classe que da acesso a ValueStruct ou ValueArray de um XmlRcpValue

	class XmlRpcValueAcess: private XmlRpcValue
	{
	public:
		XmlRpcValueAcess(XmlRpc::XmlRpcValue xml_rpc_value):XmlRpc::XmlRpcValue(xml_rpc_value){}

		XmlRpc::XmlRpcValue::ValueStruct getValueStruct()
		{
			assertStruct();
			return *_value.asStruct;
		}

		XmlRpc::XmlRpcValue::ValueArray getValueArray()
		{
			assertArray(size());
			return *_value.asArray;
		}
	};
};

};

#endif	// SSC32_SSC32_NODE_H