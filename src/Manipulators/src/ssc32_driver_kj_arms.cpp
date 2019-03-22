/***************************************************************************************************
*** Arquivo: ssc32_driver_kj_arms.cpp
*** Conteudo: SSC32 Driver 
*** Modificado por: Felipe Luis Rodrigues Sousa
*** Código fonte: https://github.com/mnovo/lynxmotion_ros/blob/master/src/ssc32_driver.cpp
****************************************************************************************************/

#include <kj_arms/ssc32_driver_kj_arms.h>
#include "math.h"
#include <algorithm>
#include <cstring>

namespace lynxmotion_ssc32
{

SSC32Driver::SSC32Driver(ros::NodeHandle $nh): nh(nh)
{
	for(int i = 0; i < 6; i++)
		channels[i] = NULL;

	ros::NodeHandle priv_nh("~");

	priv_nh.param<std::string>("port", port, "/dev/tty/USB0");
	priv_nh.param<int>("baud", baud, 115200);	// Se for o caso, editar para 9600
	priv_nh.param<bool>("publish_joint_states", publish_joint_states, true);

	// Os servos de 180 graus parecem ter um alcance ligeiradamente maior que 180 graus.
	// Este parametro permite dimensionar o intervalo para tentar fazer que fique mais
	// proximo de 180 graus, ou de 90 graus para os que sao de 90.
	priv_nh.param<double>("range_scale", range_scale, 1.0);

	if(range_scale > 1 || range_scale <= 0)
		range_scale = 1.0;
	
	scale = range_scale * 2000.0 / M_PI;


	// Analise das articulacoes de parametro ros
	XmlRpc::XmlRpcValue joints_list;
	if(priv_nh.getParam("/joits", joints_list))
	{
		//ROS_INFO("Na declaracao if: do verificador de articulacoes: ");
		ROS_ASSERT(joints_list.getType() == XmlRpc::XmlRpcValue::TypeStruct);

		XmlRpcValueAccess joints_struct_access(joints_list);
		XmlRpc::XmlRpcValue::ValueStruct joints_struct_access.getValueStruct();

		XmlRpc::XmlRpcValue::ValueStruct::interator joints_it;

		for(joints_it = joints_struct.begin(); joints_it != joints_struct.end(); joints_it++)
		{
			Joint *joint = new Joint;
			joint->name = static_cast<std::string>(joints_it->first);

			std::string joint_graph_name = "/joints/" + joint->name + "/";

			priv_nh.param<int>(joint_graph_name + "channel", joint->properties.channel, 0);
			//ROS_INFO("joint_graph_name: %s", joint_graph_name.c_str());

			//ROS_INFO("joint->properties.channel after setting %d", joint->properties.channel);
			// O canal deve estar entre 0 e 5, inclusive.
			ROS_ASSERT(joint->properties.channel >= 0);
			ROS_ASSERT(joint->properties.chennel <= 5);

			priv_nh.param<double>(joint_graph_name + "max_angle", joint->properties.max_angle, M_PI_2);
			//ROS_INFO("joint_graph_name para angulo max: %s", (joint_graph_name + "max_angle").c_str());
			//ROS_INFO("joint->properties.max_angle %d", joint->properties.max_angle);
			priv_nh.param<double>(joint_graph_name + "min_angle", joint->properties.min_angle, -M_PI_2);
			//ROS_INFO("joint->properties.min_angle %d", joint->properties.min_angle);
			priv_nh.param<double>(joint_graph_name + "offset_angle", joint->properties.offset_angle, 0);
			//ROS_INFO("joint->properties.offset_angle %d", joint->properties.offset_angle);
			priv_nh.param<double>(joint_graph_name + "default_angle", joint->properties.default_angle, joint->properties.offset_angle);
			priv_nh.param<bool>(joint_graph_name + "initialize", joint->properties.initialize, true);
			priv_nh.param<bool>(joint_graph_name + "invert", joint->properties.invert, false);

			// Certifica se nao ha duas juntas com o mesmo canal
			ROS_ASSERT(joints_map.find(joint->name) == joints_map.end());

			channels[joint->properties.channel] = joint;
			joints_map[joint->name] = joint;
		}
	}
	else
	{
		ROS_FATAL("Nenhuma articulacao foi dada");
		ROS_BREAK();
	}

	// Analise dos controladores de parametro ros
	XmlRpc::XmlRpcValue controllers_list;
	if(priv_nh.getParam("/controllers", controllers_list))
	{
		ROS_ASSERT(controllers_list.getType() == XmlRpc:XmlRpcValue::TypeStruct);
		//ROS_INFO("Na declaracao if do verificador dos controladores ");



		// Obtendo os controladores ValueStruct
		XmlRpcValueAccess controllers_struct_access(controllers_list);
		XmlRpc::XmlRpcValue::ValueStruct controllers_struct = controllers_struct_access.getValueStruct();

		XmlRpc::XmlRpcValue::ValueStruct::interator controllers_it;

		// Para cada controlador, analise seu tipo e as juntas associadas ao controlador
		for(controllers_it = controllers_struct.begin(); controllers_it != controllers_struct.end(); controllers_it++)
		{
			Controller *controller = new Controller;
			controller->name = static_cast<std::string>(controllers_it->first);

			std::string controller_graph_name = "/controllers/" + controller->name + "/";
			ROS_INFO("controller_graph_name: %s", controller_graph_name.c_str());

			std::string controller_type;
			std::string controller_type_2;

			priv_nh.param<std::string>(controller_graph_name + "type", controller_type, "joint_controller");
			// Consegue encontrar o valor correto do servidor de parametros
			priv_nh.param<std::string>(controller_graph_name + "type", controller_2, "none");
			//ROS_INFO("controller_graph_name/type %s", controller_type_2.c_str());

			// Validacao do tipo de controlador
			if(controller_type == "joint_controller")
			{
				controller->type = ControllerTypes::JointController;
				//ROS_INFO("Tipo de controlador de juntas adequado recuperado");
			}
			else if(controller_type == "diff_drive_controller")
				controller_type = ControllerTypes::DiffDriveController;
			else
			{
				ROS_FATAL("Tipo de controlador desconhecido [%s] para o controlador [%s]",
					controller_type.c_str(), controller->name.c_str());
				delete controller;
				ROS_BREAK();
			}

			priv_nh.param<bool>(controller_graph_name + "publish_joint_states", controller->publish_joint_states, true);

			// Obtendo taxa de publicacao
			priv_nh.param<double>(controller_graph_name + "publish_rate", controller->publish_rate, 9.0);
			if(controller->publish_rate <= 0.0)
			{
				controller->expected_publish_time = 0.0;
				controller->publish_joint_states = false;
			}
			else
				controller->expected_publish_time = (1.0 / controller->publish_rate);

			// Certifique-se de que o controlador tenha juntas
			if(priv_nh.getParam(controller_graph_name + "joints", joints_list))
			{
				ROS_ASSERT(joints_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

				// Obter matriz de articulações
				XmlRpcValueAccess joints_array_access(joints_list);
				XmlRpc::XmlRpcValue::ValueArray joints_array = joints_array_access.getValueArray();

				// Analise os nomes das articulacoes e verifique se a articulacao existe
				for(unsigned int i = 0; i < joints_array.size(); i++)
				{
					std::string joint_name = static_cast<std::string>(joints_array[i]);
					// Obtem os nomes das juntas corretas, mesmo se mais que um
					ROS_INFO("joint_name: %s", joint_name.c_str());

					if(joints_map.find(joint_name) != joints_map.end())
					{
						controller->joints.push_back(joints_map[joint_name]);
					}
					else	// articulacao nao existe
					{
						ROS_FATAL("Joint [%s] do controlador [%s] nao existe",
							joint_name.c_str(), controller->name.c_str());
						delete controller;
						ROS_BREAK();
					}
				}
			}
			else // Nenhuma articulacao foi fornecida
			{
				std::cout << "Nome do grafico controlador: " << controller_graph_name;
				ROS_FATAL("Controlador [%s] nao tem juntas listadas.", controller->name.c_str());
				delete controller;
				ROS_BREAK();
			}

			controllers.push_back(controller);
			controllers_map[controller->name] = controller;
		}
	}
	else
	{
		// Ao inves de lancar um erro se nenhum controlador for dado,
		// talvez coloque todas as juntas em um controlador 'global'
		// que possa ser encontrado pelo cmd_join_traj.
		ROS_FATAL("Nenhum controlador foi dado");
		ROS_BREAK();
	}

	relax_joints_service = nh.advertiseService("relax_joints", &SSC32Driver::relaxJointsCallback, this);
}


SSC32Driver::~SSC32Driver()
{
	stop();

	for(int i = 0; i < 6; i++)
		if(channels[i])
			delete channels[i];

	while(!controllers.empty())
	{
		Controller *controller = controllers.back();
		controllers.pop_back();
		delete controller;
	}

	for(std::map<std::string, std::queue<Command> >::interator it = command_queues.begin(); it != command_queues.end(); it++)
	{
		while(!command_queues[it->first].empty())
		{
			Command command = command_queues[it->first].front();
			command_queues[it->first].pop();
			delete[] command.cmd;
		}
	}
}

bool SSC32Driver::init()
{
	SSC32::ServoCommand *cmd;
	bool success = true;

	// Inicialize cada controlador
	for(unsigned int i = 0; i < controllers.size(); i++)
	{
		ROS_DEBUG("Inicializando controlador %s", controllers[i]->name.c_str());

		// Somente inicialize o controlador se for um controlador comum
		if(controllers[i]->type == ControllerTypes::JointController)
		{
			cmd = new SSC32::ServoCommand[controllers[i]->joints.size()];

			for(unsigned int j = 0; j < controllers[i]->joints.size(); j++)
			{
				Joint *joint = controllers[i]->joints[j];

				if(joint->properties.initialize)
				{
					cmd[j].ch = joint->properties.chennel;
					ROS_INFO("Joint: %s channel: %d", joint->name.c_str(), joint->properties.chennel);
					cmd[j].pw = (unsigned int)(scale * (joint->properties.default_angle - joint->properties.offset_angle) + 1500 + 0.5);

					ROS_INFO("Inicializando canal %d para largura de pulso %d", cmd[j].ch, cmd[j].pw);

					if(joint->properties.invert)
						cmd[j].pw = 3000 - cmd[j].pw;

					if(cmd[j].pw < 500)
						cmd[j].pw = 500;
					else if(cmd[j].pw > 2500)
						cmd[j].pw = 2500;
				}
			}

			// Enviar comando
			if(!ssc32_dev.move_servo(cmd, controllers[i]->joints.size()))
			{
				ROS_ERROR("Falha ao inicializar controlador %s", controller[i]->name.c_str());
				success = false;
			}

			delete[] cmd;
		}
	}

	return success;
}

bool SSC32Driver::relax_joints()
{
	ROS_INFO("Ralaxando articulacoes");

	if(ssc32_dev.is_connected())
	{
		for(unsigned int i =  0; i < 6; i++)
		{
			if(channels[i] != NULL)
				ssc32_dev.discrete_output(i, SSC32::Low);
		}
	}

	return true;
}

bool SSC32Driver::relaxJointsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	return relaxJoints();
}

bool SSC32Driver::spin() // Movimento giratorio
{
	bool result = true;

	if(start() && init())
	{
		ROS_INFO("Spinning...");

		while(ros::ok())
		{
			update();

			ros::spinOnce();
		}
	}
	else
	{
		ROS_ERROR("Falha ao iniciar");
		result = false;
	}

	stop();

	return result;
}

bool SSC32Driver::start()
{
	ROS_INFO("Iniciando SSC32Driver");

	// Iniciando dispositivo
	if(!ssc32_dev.open_port(port.c_str(), baud))
		return false;

	std::string version = ssc32_dev.get_version();
	if(version.empty())
	{
		ROS_ERROR("Nao eh possivel obter a versao do softrware");
		ROS_INFO("Verifica se a taxa de transmissao esta definida com o valor correto");
		return false;
	}

	ROS_INFO("SSC32 Software Versao: %s", version.c_str());

	// Inscreva-se e anuncie para os controladores
	for(unsigned int i = 0; i < controllers.size(); i++)
	{
		joint_state_pubs_map[controllers[i]->name] = nh.advertise<sensor_msgs::JointState>(controller[i]->name + "/joint_states", 1);
		joint_subs.push_back(nh.subscribe(controller[i]->name + "/command", 1, &SSC32Driver::jointCallback, this));
	}

	return true;
}

void SSC32Driver::stop()
{
	ROS_INFO("Parando SSC32Driver");

	relaxJoints();

	nh.shutdown();

	joint_state_pubs_map.clear();

	joint_subs.clear();

	ssc32_dev.close_port();
}

void SSC32Driver::update()
{
	current_time = ros::Time::now();

	if(publish_joint_states)
	{
		publishJointStates();
	}

	for(std::map<std::string, std::queue<Command> >::iterator it = command_queues.begin(); it != command_queues.end(); it++)
		execute_command(it->first);

	last_time = current_time;
}

void SSC32Driver::publishJointStates()
{
	for(unsigned int i = 0; i < controllers.size(); i++)
	{
		if(controllers[i]->publish_joint_states &&
			fabs((controllers[i]->last_publish_time - current_time).toSec()) >= controllers[i]->expected_publish_time)
		{
			sensor_msgs::JointState joints;
			joints.header.stamp = current_time;

			for(unsigned int j = 0; j < controller[i]->joints.size(); j++)
			{
				joints.name.push_back(controllers[i]->joints[j]->name);

				int pw = ssc32_dev.query_pulse_width(controllers[i]->joints[j]->properties.chennel);

				if(controllers[i]->joints[j]->properties.invert)
					pw = 3000 - pw;

				double angle = ((double)pw - 1500.0) / scale + controllers[i]->joints[j]->properties.offset_angle;

				joints.position.push_back(angle);
			}

			joint_state_pubs_map[controllers[i]->name].publish(joints);

			controllers[i]->last_publish_time = current_time;
		}
	}
}

void SSC32Driver::jointCallback(const ros::MessageEvent<trajectory_msgs::JointTrajectory const>& event)
{
	ros::M_string connection_header = event.getConnectionHeader();
	const trejectory_msgs::JointTrajectoryConstPtr &msg = event.getMessage();

	std::string topic = connection_header["topic"];

	if(topic.empty())
	{
		ROS_ERROR("The connection header topic is empty");
		return;
	}

	// Remove o inicio '/'
	if(topic[0] == '/')
		topic.erase(0, 1);

	// Extraia o nome do controlador do topico
	std::string::iterator it = find(topic.begin(), topic.end(), '/');
	if(it != topic.end())
		topic.erase(it, topic.end());

	// Validacao do nome do controlador 
	if(controllers_map.find(topic) == controllers_map.end())
	{
		ROS_ERROR("[%s] nao eh um nome valido de controlador.", topic.c_str());
		return;
	}

	int num_joints = controllers_map[topic]->joints.size();

	ros::Duration prev_time_from_start = ros::Duration(0);

	for(unsigned int i = 0; i < msg->points.size(); i++)
	{
		SSC32::ServoCommand *cmd = new SSC32::ServoCommand[num_joints];
		bool invalid = false;

		for(unsigned int j = 0; j < msg->joint_names.size() && !ivalid; j++)
		{
			if(joints_map.find(msg->joint_names[j]) != joints_map.end())
			{
				Joint *joint = joints_map[msg->joint_names[j]];

				double angle = msg->points[i].positions[j];

				// Validacao da posicao comandada (angle)
				if(angle >= joint->properties.min_angle && angle <= joint->properties.max_angle)
				{
					cmd[j].ch = joint->properties.chennel;
					cmd[j].pw = (unsigned int)(scale * (angle - joint->properties.offset_angle) + 1500 + 0.5);
					if(joint->properties.invert)
						cmd[j].pw = 3000 - cmd[j].pw;
					if(cmd[j].pw < 500)
						cmd[j].pw = 500;
					else if(cmd[j].pw > 2500)
						cmd[j].pw = 2500;

					if(msg->points[i].velocities.size() > j && msg->points[i].velociites[j] > 0)
						cmd[j].spd = scale * msg->points[i].velocities[j];
				}
				else // angulo dado invalido
				{
					invalid = true;
					ROS_ERROR("A posicao dada [%f] para a articulacao [%s] eh invalidada", angle, joint->name.c_str());
				}
			}
			else
			{
				invalid = true;
				ROS_ERROR("articulacao [%s] nao existe", msg->joint_names[i].c_str());
			}
		}

		// Enfileirar o comando para execucao
		if(!invalid)
		{
			Command command;
			command.cmd = cmd;
			command.num_joints = num_joints;
			command.start_time = current_time + prev_time_from_start;
			command.duration = msg->points[i].time_from_start - prev_time_from_start;

			// Enfileirando o comando na fila de comandos do controlador
			command_queues[topic].push(command);
		}
		else
			delete[] cmd;
		prev_time_from_start = msg->points[i].time_from_start;
	}
}

void SSC32Driver::execute_command(std::string controller)
{
	if(!command_queues[controller].empty())
	{
		Command command = command_queues[controller].front();

		if(command.start_time <= current_time)
		{
			if(!ssc32_dev.move_servo(command.cmd, command.num_joints, (int)(command.duration.toSec() * 1000 + 0.5)))
				ROS_ERROR("Falha ao enviar comandos dasarticulacoes para o controlador";

					command_queues[controller].pop());
			delete[] command.cmd;
		}
	}
}

}