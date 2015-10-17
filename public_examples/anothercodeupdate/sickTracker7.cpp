#include "Aria.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <fstream>
#include <unistd.h>

using namespace std;

#define D_LOG_RANGE 0.1
#define RANGE_D 0.5
#define MIN_TRACKING_DISTANCE 2000.0
#define DT 100 //Number of milleseconds used for calculating velocity

#define RUN 'r'
#define STOP 's'
#define QUIT 'q'
#define WRITE 'w'
#define NO_WRITE 'n'


#define REST 1
#define TRACKING 2
#define FOLLOWING 3


class ConnHandler
{
public:
	// Constructor
	ConnHandler(ArRobot *robot);
	// Destructor, its just empty
	~ConnHandler(void) {}
	// to be called if the connection was made
	void connected(void);
	// to call if the connection failed
	void connFail(void);
	// to be called if the connection was lost
	void disconnected(void);
protected:
	// robot pointer
	ArRobot *myRobot;
	// the functor callbacks
	ArFunctorC<ConnHandler> myConnectedCB;
	ArFunctorC<ConnHandler> myConnFailCB;
	ArFunctorC<ConnHandler> myDisconnectedCB;
};

ConnHandler::ConnHandler(ArRobot *robot) :
myConnectedCB(this, &ConnHandler::connected),  
	myConnFailCB(this, &ConnHandler::connFail),
	myDisconnectedCB(this, &ConnHandler::disconnected)

{
	myRobot = robot;
	myRobot->addConnectCB(&myConnectedCB, ArListPos::FIRST);
	myRobot->addFailedConnectCB(&myConnFailCB, ArListPos::FIRST);
	myRobot->addDisconnectNormallyCB(&myDisconnectedCB, ArListPos::FIRST);
	myRobot->addDisconnectOnErrorCB(&myDisconnectedCB, ArListPos::FIRST);
}

// just exit if the connection failed
void ConnHandler::connFail(void)
{
	printf("directMotionDemo connection handler: Failed to connect.\n");
	myRobot->stopRunning();
	Aria::shutdown();
	return;
}

// turn on motors, and off sonar, and off amigobot sounds, when connected
void ConnHandler::connected(void)
{
	printf("directMotionDemo connection handler: Connected\n");
	myRobot->comInt(ArCommands::SONAR, 0);
	myRobot->comInt(ArCommands::ENABLE, 1);
	myRobot->comInt(ArCommands::SOUNDTOG, 0);
}

// lost connection, so just exit
void ConnHandler::disconnected(void)
{
	printf("directMotionDemo connection handler: Lost connection, exiting program.\n");
	exit(0);
}

struct velocity_vector
{
	float mag;
	float rad;
	float tan;
};

struct tracking_object
{
	float distance;
	int double_degree;
	float degree;
	float vmag;
	float vrad;
	float vtan;
	float theta_dot;
	float l_edge;
	float r_edge;
};

tracking_object v_calc(tracking_object objt1, tracking_object objt2, int time)
{
	//  printf("degree t1 = %f, degree t2 = %f, dt = %i\r\n", objt1.distance, objt2.distance, time);
	objt2.vrad = (objt2.distance - objt1.distance)/(float)time;
	objt2.theta_dot = (objt2.degree - objt1.degree)/(float)time;
	objt2.vtan = (objt2.distance + objt1.distance)*0.5*(objt2.degree-objt1.degree)*(3.1415/180.0)/(float)time;
	objt2.vmag = sqrt((objt2.vtan*objt2.vtan)+(objt2.vrad*objt2.vrad));
	return(objt2);
}

float r_diff(tracking_object objt1, tracking_object objt2)
{
	float d_rad = objt2.distance - objt1.distance;
	float d_tan = (objt2.distance + objt1.distance)*0.5*(objt2.degree - objt1.degree)*(3.1415/180.0);

	float diff = sqrt((d_rad*d_rad)+(d_tan*d_tan));

	return(diff);
}


int min_range(ArSick * sick)
{
	char empty_flag = 0;
	char null_flag = 0;

	int min_range_val = 999999;
	int range = 0;
	std::list<ArSensorReading *> *r1;
	std::list<ArSensorReading *>::iterator it;

	(*sick).lockDevice();
	r1=(list<ArSensorReading *,allocator<ArSensorReading *> > *)(*sick).getRawReadings();
	(*sick).unlockDevice();

	// printf("finished scan %i of %i\r\n",j,num_scans);

	if(NULL!=r1)
	{
		if (r1->end() != r1->begin())
		{ 
			// Process and store data from first reading
			for (it = r1->begin(); it!= r1->end(); it++)
			{
				range = ((*it)->getRange());
				if (range < min_range_val)
					min_range_val = range;
			}

		}
		else
		{
			empty_flag = 1;
		}
	}
	else
	{
		null_flag = 1;
	}

	if(null_flag || empty_flag)
		return -1;

	return(min_range_val);
}


std::vector<tracking_object> run_sick_scan(ArSick * sick, int num_scans, char file_write_option = 0, int upper_angle = 350, int lower_angle = 10)
{

	if(upper_angle > 350)
		upper_angle = 350;
	if(lower_angle < 10)
		lower_angle = 10;
	if((upper_angle - lower_angle) < 60)
	{
		upper_angle = 350;
		lower_angle = 10;
	}


	std::vector<tracking_object> objects1;
	float range1 [361] = {0};
	float log_range [361] = {0};
	float drange1 [360] = {0};
	float d_log_range [360] = {0};

	int i = 0;
	char null_flag = 0;
	char empty_flag = 0;

	for (int j = 0; j < num_scans; j++)
	{
		std::list<ArSensorReading *> *r1;
		std::list<ArSensorReading *>::iterator it;

		// printf("beginning scan %i of %i\r\n",j,num_scans);

		(*sick).lockDevice();
		r1=(list<ArSensorReading *,allocator<ArSensorReading *> > *)(*sick).getRawReadings();
		(*sick).unlockDevice();

		// printf("finished scan %i of %i\r\n",j,num_scans);

		if(NULL!=r1)
		{
			i = 0;
			if (r1->end() != r1->begin())
			{ 
				// Process and store data from first reading
				for (it = r1->begin(); it!= r1->end(); it++)
				{
					range1[i] = range1[i] + ((*it)->getRange());
					i++;
				}

			}
			else
			{
				empty_flag = 1;
			}
		}
		else
		{
			null_flag = 1;
		}
	}
	/////////////////////////////////

	ofstream raw_output_file;
	

	if(!(null_flag && empty_flag))
	{
		if(file_write_option == 'w')
		{
			raw_output_file.open("./scan_data/most_recent_scan.txt");
		}

		for(i = lower_angle; i < upper_angle; i++)
		{
			range1[i] = range1[i]/(float)(num_scans + 1);
			
			log_range[i] = log(range1[i]);
			if(i)
			{
				drange1[i-1] = range1[i] - range1[i-1];
				d_log_range[i-1] = log_range[i]-log_range[i-1];


				if(file_write_option == 'w')
				{
					raw_output_file << (float)i/2.0;
					raw_output_file<< "\t";
					raw_output_file << range1[i];
					raw_output_file << "\t";
					raw_output_file << drange1[i-1];
					raw_output_file << "\t";
					raw_output_file << log_range[i];
					raw_output_file << "\t";
					raw_output_file << d_log_range[i-1];
					raw_output_file << "\r\n";
				}
			}
		}
		if(file_write_option == 'w')
		{
			raw_output_file.close();
		}

		i = lower_angle;

		int r_edge = 0;
		int l_edge = 0;

		// Cycle through first set of readings and identify objects
		while(i < upper_angle)
		{
			//This if statement looks for the left edge of an object, which shows up as a negative drange
			char exit_flag = 0;
			if(d_log_range[i] < - D_LOG_RANGE)
			{
				if(range1[i+1] < MIN_TRACKING_DISTANCE)
				{
					r_edge = i+4;
					while((d_log_range[i] < D_LOG_RANGE)&& !exit_flag)
					{
						i++;
						if(d_log_range[i+1] < -D_LOG_RANGE)
						{
							exit_flag = 1;
							i = i - 1;
							//printf("Exit flag at i = %i\r\n",i);
						}
					}
					l_edge = i-4;
				        //printf("object: left edge %i\tright edge %i\r\n", l_edge, r_edge);
					if(((i < upper_angle)&&((l_edge - r_edge)>10))&& !exit_flag)
					{
						float average_range = 0;
						tracking_object new_obj;
						new_obj.double_degree = (r_edge + (l_edge - r_edge)/2);
						new_obj.l_edge = l_edge/2.0;
						new_obj.r_edge = r_edge/2.0;

						for(int j = r_edge; j < l_edge; j++)
						{
							average_range = average_range + range1[j];
						}

						average_range = average_range/(l_edge - r_edge);

						new_obj.degree = new_obj.double_degree/2.0;
						new_obj.distance = average_range;
						objects1.push_back(new_obj);
					}
				}

			}
			i++;
		}
	}
	return(objects1);
}





std::vector<tracking_object> get_moving_objects(ArSick * sick, int ms_time, int num_scans, char user_command, int upper_angle = 10, int lower_angle = 350)
{
	std::vector<tracking_object> objects1;
	std::vector<tracking_object> objects2;
	std::vector<tracking_object> obj_vector;

	ArTime start;

	do
	{
		start.setToNow();

		objects1 = run_sick_scan(sick, num_scans);
		usleep(ms_time*1000);
		objects2 = run_sick_scan(sick, num_scans, user_command);

		ms_time = start.mSecSince();
	} while (objects1.size() != objects2.size());

	unsigned int i = 0;
	unsigned int j = 0;

	ofstream obj_file;
	ofstream vel_file;

	if(user_command == 'w')
	{
		obj_file.open("./scan_data/detected_objs.txt");
		vel_file.open("./scan_data/obj_vels.txt");
	}


	for(j = 0; j < objects2.size(); j++)
	{
		tracking_object v;
		tracking_object new_v;
		v = objects2[j];
		v.vmag = 99999.9;
		for(i = 0; i < objects1.size(); i++)
		{
			new_v = v_calc(objects1[i],objects2[j],ms_time);

			if (new_v.vmag < v.vmag)
			{
				v = new_v;
			}
		}
		
		if(v.vmag > 5.0)
		{
			v.vmag = 0.0;
			v.vrad = 0.0;
			v.vtan = 0.0;
			v.theta_dot = 0.0;
		}
		

		if(user_command == 'w')
		{
			obj_file << "\r\n";
			obj_file << v.r_edge;
			obj_file<< "\t";
			obj_file << v.distance;
			obj_file<< "\t";
			obj_file << log(v.distance);
			obj_file << "\r\n";

			obj_file << v.l_edge;
			obj_file<< "\t";
			obj_file << v.distance;
			obj_file<< "\t";
			obj_file << log(v.distance);
			obj_file << "\r\n";

			vel_file << "\r\n";
			vel_file << v.degree;
			vel_file << "\t";
			vel_file << v.distance;
			vel_file << "\t";
			vel_file << v.vrad * 1000.0;
			vel_file << "\t";
			vel_file << v.theta_dot;
			vel_file << "\r\n";
		}


		obj_vector.push_back(v);

	}
	if(user_command == 'w')
	{
		obj_file.close();
		vel_file.close();
	}
	return(obj_vector);
}


int print_object(tracking_object obj)
{

	printf("Distance %f\r\n", obj.distance);
	printf("Degree %f\r\n", obj.degree);
	printf("V mag %f\r\n", obj.vmag);
	printf("V rad %f\r\n", obj.vrad);
	printf("V tan %f\r\n", obj.vtan);
	return(0);
}



int main(int argc, char **argv) 
{
	ArRobot robot;
	Aria::init();
	//laser
	int ret; //Don't know what this variable is for
	ArSick sick; // Laser scanner
	ArSerialConnection laserCon; // Scanner connection
	std::string str; // Standard output
	// sonar, must be added to the robot
	ArSonarDevice sonar;
	// add the sonar to the robot
	robot.addRangeDevice(&sonar);
	// add the laser to the robot
	robot.addRangeDevice(&sick);

	ArArgumentParser argParser(&argc, argv);
	ArSimpleConnector con(&argParser);

	// the connection handler from above
	ConnHandler ch(&robot);

	if(!Aria::parseArgs())
	{
		Aria::logOptions();
		Aria::shutdown();
		return 1;
	}
	if(!con.connectRobot(&robot))
	{
		ArLog::log(ArLog::Normal, "directMotionExample: Could not connect to the robot. Exiting.");
		return 1;
	}
	ArLog::log(ArLog::Normal, "directMotionExample: Connected.");
	robot.runAsync(false);

	///////////////////////////////
	// Attempt to connect to SICK using another hard-coded USB connection
	sick.setDeviceConnection(&laserCon);
	if((ret=laserCon.open("/dev/ttyUSB1")) !=0){
		//If connection fails, shutdown
		Aria::shutdown();
		return 1;
	}
	//Configure the SICK
	sick.configureShort(false,/*not using sim*/ArSick::BAUD38400,ArSick::DEGREES180,ArSick::INCREMENT_HALF);
	//Run the sick
	sick.runAsync();
	// Presumably test to make sure that the connection is good
	if(!sick.blockingConnect()){
		printf("Could not get sick...exiting\n");
		Aria::shutdown();
		return 1; 
	}
	printf("We are connected to the laser!");

	printf("\r\nRobot Entering default resting state.\r\nUse the following commands to run the robot.\r\n");
	printf("r  Run the robot\r\n");
	printf("s  Stop the robot\r\n");
	printf("w  Save current data to files, and show a plot of what the robot sees.\r\n");
	printf("q  Quit the program\r\n");

	printf("\r\n");
	printf("\r\n");
	printf("NOTE: You must have GNUPLOT installed on your computer, and create a directory entitled 'scan_data' under your current directory in order to use this script. Failure to do so might make the computer crash, which would cause the robot to go on a mad killing spree! Not really, but seriously, go ahead and get GNUPLOT and create that subdirectory before using the 'w' option.\r\n\r\n"); 

	/////////////////////////////////////
	char user_command = 0;
	char plot_option = 0;

	int robot_state = REST;

	tracking_object target;



	std::vector<tracking_object> obj_vector;
	while(user_command != 'q')
	{
		switch (user_command)
		{
		case STOP:
			robot_state = REST;
			printf("robot has entered resting mode\r\n");

			break;
		case RUN:
			robot_state = TRACKING;
			printf("robot has entered tracking mode\r\n");
			break;
		case QUIT:
			robot_state = -1;
			printf("exiting... goodbye.\r\n");
			break;
		case WRITE:
			plot_option = WRITE;
			break;
		case NO_WRITE:
			plot_option = NO_WRITE;
			break;

		default:
			robot_state = robot_state;
		}


		unsigned int i = 0;
		unsigned int num_objects = obj_vector.size();
		int tracking_obj_index = -1;
		float min_distance = 999999.9;
		float new_heading = 0;

		switch (robot_state)
		{


		case REST:
			obj_vector = get_moving_objects(&sick, DT, 10, plot_option);
			break;

		case TRACKING:
			obj_vector = get_moving_objects(&sick, DT, 10, plot_option);
			
			if(num_objects)
			{
				for(i = 0; i < obj_vector.size();i++)
				{	
					if((obj_vector[i].vmag > 0.2)&&(obj_vector[i].vmag<2.0))
					{	
						print_object(obj_vector[i]);			  
						if(obj_vector[i].distance < min_distance)
						{
							min_distance = obj_vector[i].distance;
							target = obj_vector[i];
							tracking_obj_index = i;
						}
					}
				}

				if(tracking_obj_index > -1)
				{
					int l_edge = obj_vector[tracking_obj_index].l_edge;
					int r_edge = obj_vector[tracking_obj_index].r_edge;
// Do another scan to make sure the object is actually there.

					if(num_objects)
					{
						obj_vector = get_moving_objects(&sick, DT, 10, 'w', r_edge - 10, l_edge + 10);
						tracking_obj_index = 0;
						for(i = 0; i < obj_vector.size();i++)
						{	

							if((obj_vector[i].vmag > 0.2)&&(obj_vector[i].vmag<2.0))
							{	
								print_object(obj_vector[i]);			  
								if(obj_vector[i].distance < min_distance)
								{
									min_distance = obj_vector[i].distance;
									tracking_obj_index = i;
								}
							}
						}
						if(tracking_obj_index > -1)
						{
							new_heading = -90 + obj_vector[tracking_obj_index].degree;

							printf("delta heading confirmed, setting to %f\r\n", new_heading);
							robot.lock();
							robot.setDeltaHeading(new_heading);
							robot.unlock();
							usleep(abs((int)new_heading*30*1000 + 50000));

							robot_state = FOLLOWING;
							printf("entering following mode!\r\n");
							target = obj_vector[tracking_obj_index];
							target.degree = target.degree - new_heading;

						}
					}
				}		      
			}
			break;

		case FOLLOWING:
			obj_vector = get_moving_objects(&sick, DT, 10, WRITE, 10, 350);
			i = 0;
			num_objects = obj_vector.size();
			int to_ind = -1;
			float min_distance = 999999.9;
			float obj_difference = 9999999.9;

			if(num_objects)
			{
				for(i = 0; i < obj_vector.size();i++)
				{	
					if(r_diff(target, obj_vector[i]) < obj_difference)
					{	

						if(obj_vector[i].distance < min_distance)
						{
							min_distance = obj_vector[i].distance;
							to_ind = i;
						}
					}
				}
			}
			if(min_range(&sick) > 500)
			{
				if(to_ind > -1)
				{
					target = obj_vector[to_ind];

//					system("./scan_data/plot_script.sh >/dev/null");
					float new_vel = (obj_vector[to_ind].distance) - 500;
					if(new_vel < 0)
						new_vel = 0;

					if(new_vel > 500)
						new_vel = 500;				

					robot.lock();
					robot.setVel(new_vel);
					robot.unlock();

					new_heading = -90 + target.degree;

					robot.lock();
					robot.setDeltaHeading(new_heading);
					robot.unlock();
					target.degree = target.degree - new_heading;
				}
			}
			else
			{
				robot.lock();
				robot.setVel(0);
				robot.unlock();
				robot_state = TRACKING;
			}

/*
*/


		}

		if(plot_option == WRITE)
		{
			system("./scan_data/plot_script.sh >/dev/null");
		}







////////////////////////////////////////////////////////////////////// 
// Everything past this point is code to grab the user input
		user_command = 0;


		fd_set rfds;
		struct timeval tv;
		int retval;

		FD_ZERO(&rfds);
		FD_SET(0, &rfds);

		tv.tv_sec = 0;
		tv.tv_usec = 100;

		retval = select(1, &rfds, NULL, NULL, &tv);

		if(retval == -1)
			perror("select()");
		else if(retval)
		{
			cin >> user_command;
			printf("input detected from user\r\n");
		}

		plot_option = NO_WRITE;
//////////////////////////////////////////////////////////////////////
	}

	Aria::shutdown();
	printf("Shutting down");
	return 0;
}
