/***************************************************************************************************
*** Arquivo: ssc32_node_kj_arms.cpp
*** Conteudo: SSC32
*** Modificado por: Felipe Luis Rodrigues Sousa
*** Código fonte: https://github.com/smd-ros-devel/lynxmotion_ssc32/blob/master/src/ssc32_node.cpp
****************************************************************************************************/


#include <ros/ros.h>
#include "kj_arms/ssc32_driver_kj_arms.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "ssc32_node" );
	ros::NodeHandle nh;

	lynxmotion_ssc32::SSC32Driver ssc32( nh );

	ssc32.spin( );

	return 0;
}