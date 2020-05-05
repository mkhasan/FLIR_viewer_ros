/*
 * FLIR_viewer_node.cpp
 *
 *  Created on: Feb 28, 2020
 *      Author: kict
 */



#include "FLIR_viewer/FLIR_viewer.h"




#include "ros/ros.h"
#include "std_msgs/String.h"

#include "client_interface/shm_manager.h"
#include "FLIR_viewer/FLIR_viewer.h"

#include <vector>


using namespace std;

static int quit=0;






void mySigintHandler(int sig)
{

	ROS_WARN("Signal %d caught", sig);

	if(sig == SIGINT) {
		ROS_WARN("SIGINT caught");


	}


	quit = 1;
	//pthread_join( cam_viewer_thread, NULL);


	//ros::Duration(1.0).sleep();
	//ros::shutdown();
}

int main(int argc, char **argv) {


	ros::init(argc, argv, "FLIR_viewer_node", ros::init_options::NoSigintHandler);

	ros::NodeHandle nh;

	int delay_on_start=0;
	int id=0;



	nh.param("/CAM_VIEWER/delay_on_start", delay_on_start, 0);

	if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
	   ros::console::notifyLoggerLevelsChanged();
	}

	string mutexPrefix;
	nh.param("/ROBOT/MUTEX_PREFIX", mutexPrefix, string("kict_mp_camera00_"));

	ROS_ERROR("MUTEX PREFIX IS %s", mutexPrefix.c_str());

	signal(SIGTERM, mySigintHandler);
	signal(SIGINT, mySigintHandler);          // caught in a different way fo$
	signal(SIGHUP, mySigintHandler);
	signal(SIGKILL, mySigintHandler);
	signal(SIGTSTP, mySigintHandler);



	for (int k=10*delay_on_start; k>0; k--) {

		ROS_DEBUG("FLIR: wait for %d more secs", k);
		ros::Duration(1.0).sleep();

	}

	vector<int> ids {3, 4};




	boost::shared_ptr<FLIR_Viewer> cap;
	try {

		cap.reset(new FLIR_Viewer(ids, mutexPrefix));

		while (ros::ok() && quit==0) {


			ros::spinOnce();

			ros::Duration(5.0).sleep();

			ROS_DEBUG("ing ...");
		}



	}
	catch (exception &e) {
		ROS_ERROR("Error: %s", e.what());

		//cap1.Finalize();
		ros::shutdown();
	}


	ROS_DEBUG("Terminating");





	return 0;
}


