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


#define PI 3.14159265

using namespace std;

#define D_LOG_RANGE 0.1
#define RANGE_D 0.5
#define MIN_TRACKING_DISTANCE 5000.0
#define DT 100 //Number of milleseconds used for calculating velocity
#define MAX_V 750

#define ROBOT_SAFETY_MARGIN 250.0

#define NO_ERROR 0
#define EMPTY_DATA 1
#define NULL_DATA 2


#define RUN 'r'
#define STOP 's'
#define QUIT 'q'
#define WRITE 'w'
#define NO_WRITE 'n'
#define TEST 't'
#define EDIT 'e'


#define REST 1
#define TRACKING 2
#define FOLLOWING 3
#define TOO_CLOSE 4

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



int go_to_position(ArRobot * robot, ArSick * sick, tracking_object position, float v = 300.0)
{

	(*robot).lock();
	(*robot).setDeltaHeading(position.degree - 90);
	(*robot).unlock();

	ArUtil::sleep(10);

	long int drive_time = (position.distance * 1000)/(v);
	ArTime start;

	(*robot).lock();
	(*robot).setVel(v);
	(*robot).unlock();


	start.setToNow();
	while(start.mSecSince() < drive_time)
	{
		;
	}

	(*robot).lock();
	(*robot).setVel(0);
	(*robot).unlock();

	return(0);
}




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


float is_path_safe(int range [361], float distance, float dtheta = 0.0)
{
	int i = 0;
	int unsafe_angle = 0;//It is not used
	float theta = 0;
	while((i < 361)&&(unsafe_angle == 0))
	{
		theta = (float)i/2.0;
		if((range[i] * sin((theta - dtheta)*PI/180.0)) < distance)
		{
			if((range[i] * cos((theta - dtheta)*PI/180.0)) < ROBOT_SAFETY_MARGIN)
			{
				return(i/2.0);
			}
		}
		i++;
	}

	return (-1);
}


float get_safe_path(ArSick * sick, float new_heading, float distance)
{
	float safe_path;
	float turn_direction;

	int range [361] = {0};
	int i = 0;
	char error_flag = 0;
	std::list<ArSensorReading *> *r1;
	std::list<ArSensorReading *>::iterator it;

	(*sick).lockDevice();
	r1=(list<ArSensorReading *,allocator<ArSensorReading *> > *)(*sick).getRawReadings();
	(*sick).unlockDevice();

	do{
		if(NULL!=r1)
		{
			if (r1->end() != r1->begin())
			{ 
				// Process and store data from first reading
				for (it = r1->begin(); it!= r1->end(); it++)
				{
					range[i] = ((*it)->getRange());
				}
			}
			else
			{
				error_flag = EMPTY_DATA;
			}
		}
		else
		{
			error_flag = NULL_DATA;
		}
	} while(error_flag);

	safe_path = is_path_safe(range, distance, new_heading);

	if(safe_path > 0)//have obstacle
	{
		if(safe_path < 90)
		{
			turn_direction = 1.0;
		}
		else
		{
			turn_direction = -1.0;
		}
		while(safe_path > 0)//never enter this function
		{
			new_heading = new_heading + turn_direction;
			printf("Testing heading %f...\r\n", turn_direction);
			safe_path = is_path_safe(range, new_heading, distance);
			if((new_heading > 90)||(new_heading < -90))
				return(-180);
		}
	}
	return(new_heading);
}



int min_range(ArSick * sick, float min_degree = 0., float max_degree = 180.)
{
	char empty_flag = 0;
	char null_flag = 0;

	int min_range_val = 999999;
	int range = 0;

	int min_degree_int = (min_degree * 2) - 1;
	int max_degree_int = (max_degree * 2) + 1;

	int degree_counter = 0;

	std::list<ArSensorReading *> *r1;
	std::list<ArSensorReading *>::iterator it;

	(*sick).lockDevice();
	r1=(list<ArSensorReading *,allocator<ArSensorReading *> > *)(*sick).getRawReadings();
	(*sick).unlockDevice();

	if(NULL!=r1)
	{
		if (r1->end() != r1->begin())
		{ 
			// Process and store data from first reading
			for (it = r1->begin(); it!= r1->end(); it++)
			{
				range = ((*it)->getRange());
				if((degree_counter > min_degree_int)&&(degree_counter < max_degree_int))
				{
					if (range < min_range_val)
						min_range_val = range;
				}
				degree_counter++;
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
	char error_flag;

	std::list<ArSensorReading *> *r1;
	std::list<ArSensorReading *>::iterator it;

	for (int j = 0; j < num_scans; j++)
	{

		do
		{
			error_flag = 0;
			(*sick).lockDevice();
			r1=(list<ArSensorReading *,allocator<ArSensorReading *> > *)(*sick).getRawReadings();
			(*sick).unlockDevice();

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
					error_flag = EMPTY_DATA;
				}
			}
			else
			{
				error_flag = NULL_DATA;
			}
		}while(error_flag);
	}
	/////////////////////////////////

	ofstream raw_output_file;
	
 
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
				raw_output_file << "\t";
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
				r_edge = i+2;
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
				l_edge = i-2;
				//printf("object: left edge %i\tright edge %i\r\n", l_edge, r_edge);
				if(((i < upper_angle)&&((l_edge - r_edge)>5))&& !exit_flag)
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
	return(objects1);
}





std::vector<tracking_object> get_moving_objects(ArSick * sick, int ms_time, int num_scans, char user_command, int upper_angle = 10, int lower_angle = 350)
{
	std::vector<tracking_object> objects1;
	std::vector<tracking_object> objects2;
	std::vector<tracking_object> obj_vector;
	int t_elapsed;

	ArTime start;
	do
	{
		start.setToNow();

		objects1 = run_sick_scan(sick, num_scans);
		usleep(ms_time*1000);
		objects2 = run_sick_scan(sick, num_scans, user_command);

		t_elapsed = start.mSecSince();
	}while((objects1.size() != objects2.size())&&(objects1.size() > 0));

	unsigned int i = 0;
	unsigned int j = 0;

	for(j = 0; j < objects2.size(); j++)
	{
		tracking_object v;
		tracking_object new_v;
		v = objects2[j];
		v.vmag = 99999.9;
		for(i = 0; i < objects1.size(); i++)
		{
			new_v = v_calc(objects1[i],objects2[j],t_elapsed);

			if (new_v.vmag < v.vmag)
			{
				v = new_v;
			}
		}
		
		if(v.vmag > 3.0)
		{
			v.vmag = 0.0;
			v.vrad = 0.0;
			v.vtan = 0.0;
			v.theta_dot = 0.0;
		}

		obj_vector.push_back(v);

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

int print_object_to_stream(tracking_object v, std::ostream& file)
{
	file << "\r\n";
	file << v.r_edge;
	file << "\t";
	file << v.distance;
	file << "\t";
	file << log(v.distance);
	file << "\r\n";

	file << v.l_edge;
	file << "\t";
	file << v.distance;
	file << "\t";
	file << log(v.distance);
	file << "\t";

	file << v.degree;
	file << "\t";
	file << v.vrad * 1000.0;
	file << "\t";
	file << v.theta_dot;
	file << "\r\n";

	return(0);
}

struct settings
{
	float max_velocity;
	float min_distance;
	float tracking_factor;
};


int edit_settings(settings * current_settings)
{
	int user_option;

	printf("\r\n");
	printf("You have entered the robot settings menu.\r\n");
	printf("NOTE!!! THIS CODE WILL NOT GUARD AGAINST STUPID USERS! IF YOU DON'T KNOW WHAT YOU'RE DOING, DON'T MESS WITH THE SETTINGS!!!\r\n\r\n");
	printf("Please use this menu to select which setting you would like to edit.\r\n");
	printf("\r\n");
	printf("1\tMaximum Velocity\r\n");
	printf("2\tMinimum Following Distance\r\n");
	printf("3\tRobot \"Tracking Factor\"\r\n");
	printf("0\tExit menu\r\n");

	scanf("%i", &user_option);
	printf("\r\n\r\n");

	switch(user_option)
	{
	case 1:
		printf("Current maximum velocity is set to %f\r\n", (*current_settings).max_velocity);
		printf("Enter new desired velocity:\r\n");
		scanf("%f", &((*current_settings).max_velocity));
		printf("\r\nMaximum velocity now set to %f\r\n", (*current_settings).max_velocity);
		break;
	case 2:
		printf("Current minimum following distance is set to %f\r\n", (*current_settings).min_distance);
		printf("Enter new desired minimum following distance:\r\n");
		scanf("%f", &((*current_settings).min_distance));
		printf("\r\nMinimum distance now set to %f\r\n", (*current_settings).min_distance);
		break;
	case 3:
		printf("Tracking factor is used as follows:\r\n velocity = TF*(distance - min_distance)\r\n");
		printf("Current Tracking Factor set to %f\r\n", (*current_settings).tracking_factor);
		printf("Suggest tracking factor less than 3\r\n");
		printf("Enter new desired tracking factor:\r\n");
		scanf("%f", &((*current_settings).tracking_factor));
		printf("\r\nTracking factor now set to %f\r\n", (*current_settings).tracking_factor);
		break;
	default:
		return(0);
	}

	printf("Would you like to edit any other parameters?\r\n");
	printf("1\tYes\r\n");
	printf("2\tNo\r\n");
	scanf("%i", &user_option);

	if(user_option == 1)
		edit_settings(current_settings); 

	return(0);
}



int main(int argc, char **argv) 
{
	struct settings robot_settings;

	robot_settings.min_distance = 500;
	robot_settings.max_velocity = 500;
	robot_settings.tracking_factor = 1.0;

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
	printf("t  Enter test mode (the robot will do everything except actually move.\r\n");
	printf("w  Save current data to files, and show a plot of what the robot sees.\r\n");
	printf("e  Edit robot control parameters\r\n");
	printf("q  Quit the program\r\n");

	printf("\r\n");
	printf("\r\n");
	printf("NOTE: You must have GNUPLOT installed on your computer, and create a directory entitled 'scan_data' under your current directory in order to use this script. Failure to do so might make the computer crash, which would cause the robot to go on a mad killing spree! Not really, but seriously, go ahead and get GNUPLOT and create that subdirectory before using the 'w' option.\r\n\r\n"); 

	/////////////////////////////////////
	char user_command = 0;
	char plot_option = 0;

	int robot_state = REST;

	tracking_object target;
	tracking_object l_target;

	ofstream fobjects;
	ofstream ftarget;
	ofstream flog;
	ofstream fltarget;


	fobjects.open("./scan_data/objects_new.txt");
	ftarget.open("./scan_data/target_new.txt");
	fltarget.open("./scan_data/ltarget_new.txt");
	fobjects << "\r\n";
	ftarget << "\r\n";
	fltarget << "\r\n";
	fobjects.close();
	ftarget.close();
	fltarget.close();

	flog.open("robot_log.txt");

	std::vector<tracking_object> obj_vector;
	std::vector<tracking_object> new_vector;


	float last_v = 0;

	ArTime start;



	char test_flag = 0;
	char target_lost = 0;


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
		case TEST:
			if(test_flag == TEST)
			{
				test_flag = 0;
				printf("Exiting test mode\r\n");
			}
			else
			{
				test_flag = TEST;
				printf("Entering test mode\r\n");
				robot.lock();
				robot.setVel(0);
				robot.unlock();
			}
			break;

		case EDIT:
			edit_settings(&robot_settings);
			break;

		default:
			robot_state = robot_state;
		}


		unsigned int i = 0;
		unsigned int num_objects = 0;
		int to_ind = -1;
		float min_distance = 999999.9;
		float new_heading = 0;

		system("mv ./scan_data/objects_new.txt ./scan_data/objects.txt");
		system("mv ./scan_data/target_new.txt ./scan_data/target.txt");
		system("mv ./scan_data/ltarget_new.txt ./scan_data/ltarget.txt");

		fobjects.open("./scan_data/objects_new.txt");
		ftarget.open("./scan_data/target_new.txt");
		fltarget.open("./scan_data/ltarget_new.txt");

		switch (robot_state)
		{
		case REST:
			robot.lock();
			robot.setVel(0);
			robot.unlock();

			obj_vector = run_sick_scan(&sick, 2, plot_option);

			target_lost = 0;
			num_objects = obj_vector.size();
			if(num_objects > 0)
			{

				for(i = 0; i < num_objects; i++)
				{
					print_object_to_stream(obj_vector[i], fobjects);	
				}
			}

			break;

		case TRACKING:
			flog << "TRACKING\r\n";

			obj_vector = get_moving_objects(&sick, 2*DT, 10, plot_option);
			num_objects = obj_vector.size();

			target_lost = 0;

			if(min_range(&sick, 45, 135) > ROBOT_SAFETY_MARGIN)
			{
				if(num_objects)
				{
 					for(i = 0; i < obj_vector.size();i++)
					{	
						print_object_to_stream(obj_vector[i], fobjects);
						if(obj_vector[i].vmag > 0.1)
						{	
							if(obj_vector[i].distance < min_distance)
							{
								min_distance = obj_vector[i].distance;
								target = obj_vector[i];
								to_ind = i;
							}
						}
					}

					if(to_ind > -1)
					{
						int l_edge = target.l_edge;
						int r_edge = target.r_edge;
						float difference = 9999999.9;

// Do another scan to make sure the object is actually there.
						new_vector = get_moving_objects(&sick, DT, 10, 0, 5, 175);

						num_objects = new_vector.size();
						to_ind = -1;
						if(num_objects)
						{
							for(i = 0; i < num_objects;i++)
							{
								if(new_vector[i].vmag > 0.1)
								{	
									if(r_diff(target, new_vector[i])<difference)
									{
										difference = r_diff(target, new_vector[i]);
										if(difference < 500.0)
											to_ind = i;
									}
								}
							}
						}

						if(to_ind > -1)
						{
							new_heading = (-90 + new_vector[to_ind].degree);

							if(test_flag != TEST)
							{
								robot.lock();
								robot.setDeltaHeading(new_heading);
								robot.unlock();
							}

							robot_state = FOLLOWING;
							target = new_vector[to_ind];

							target.degree = target.degree - new_heading;


							print_object_to_stream(target, ftarget);

							print_object_to_stream(target, flog);
							flog << new_heading;
						}

					}
				}
			}
			else		      
			{
				robot_state = TOO_CLOSE;
				printf("I'm too close to an obstacle, and I'm getting claustrophobic! I'm going to slowly back up now.\r\n");
			}



			break;

		case FOLLOWING:
		{
			flog << "FOLLOWING\r\n";

			i = 0;
			int to_ind = -1;
			float obj_difference = 9999999.9;
			int num_scans = 0;

			if(min_range(&sick, 45, 135) > ROBOT_SAFETY_MARGIN)
			{
				while((num_scans < 3)&&(to_ind == -1))
				{
					obj_vector = run_sick_scan(&sick, 2, 0, 10, 350);
					num_objects = obj_vector.size();
					if(num_objects)
					{
						for(i = 0; i < num_objects;i++)
						{	
							if(r_diff(target, obj_vector[i]) < obj_difference)
							{	
								obj_difference = r_diff(target, obj_vector[i]);
								if (obj_difference < 500.0)
									to_ind = i;

							}
						}

						for(i = 0; i < num_objects;i++)
						{	
							if((int)i == to_ind)
							{
								print_object_to_stream(obj_vector[i], ftarget);
							}
							else
							{
								print_object_to_stream(obj_vector[i], fobjects);
							}
						}
					}
					num_scans++;
				}

				float new_vel = 0;

				if(to_ind > -1)
				{
					printf("Tracking target\r\n");
					flog << "Following target\t";

					target_lost = 0;

					target = obj_vector[to_ind];

					new_vel = (obj_vector[to_ind].distance - robot_settings.min_distance)*robot_settings.tracking_factor;
					if(new_vel < 0)
						new_vel = 0;

					if(new_vel > robot_settings.max_velocity)
						new_vel = robot_settings.max_velocity;

					new_heading = (-90 + target.degree)*0.25;
					new_heading = get_safe_path(&sick, new_heading, target.distance);

					if(test_flag != TEST)
					{
						robot.lock();
						robot.setVel(new_vel);
						robot.unlock();

						last_v = new_vel;

						robot.lock();
						robot.setDeltaHeading(new_heading);
						robot.unlock();
						target.degree = target.degree - new_heading;
					}
					if(new_vel < 1.0)
					{
						robot_state = TRACKING;
						printf("entering tracking mode\r\n");
						flog<<"entering tracking mode\r\n";
					}

				}
				else		      
				{
					if(target_lost)
					{
						
						l_target.distance = l_target.distance - last_v*(start.mSecSince()/1000.0);
 
						int temp_max_v = min_range(&sick, 5, 175);
						if(temp_max_v < last_v)
							last_v = temp_max_v;

						if(last_v > robot_settings.max_velocity)
							last_v = robot_settings.max_velocity;

						if(test_flag != TEST)
						{

							robot.lock();
							robot.setVel(last_v);
							robot.unlock();


							new_heading = -90.0 + l_target.degree;
							new_heading = get_safe_path(&sick, new_heading, l_target.distance);

							l_target.degree = target.degree - new_heading;

							robot.lock();
							robot.setDeltaHeading(new_heading);
							robot.unlock();
						}

						print_object_to_stream(l_target, fltarget);



					}
					else
					{
						target_lost = 1;
						l_target = target;
					}

					printf("target lost\r\n");
					flog << "target lost\r\n";
					start.setToNow();

					if(target.distance < ROBOT_SAFETY_MARGIN)
					{
						robot_state = TRACKING;
						target_lost = 0;
					}

					target_lost = 1;

				}
				printf("Velocity: %f\t",last_v);
				flog << "Velocity:\t";
				flog << last_v;

				printf("Heading: %f\t",new_heading);
				flog << "\tHeading\t";
				flog << new_heading;
				flog << "\r\n";
				printf("\r\n");

			}		
			else		      
			{
				robot_state = TOO_CLOSE;
				printf("I'm too close to an obstacle, and I'm getting claustrophobic! I'm going to slowly back up now.\r\n");
			}

			break;



		}

		case TOO_CLOSE:
		{
			flog << "Too close\r\n";
			if(min_range(&sick, 45, 135) > (ROBOT_SAFETY_MARGIN + 100))
			{		
				robot_state = TRACKING;
				printf("I feel better now! I'm going to reenter tracking mode.\r\n");
				robot.lock();
				robot.setVel(0);
				robot.unlock();
			}
			else
			{
				if(test_flag != TEST)
				{
					printf("Backing up...\r\n");
					robot.lock();
					robot.setVel(-50);
					robot.unlock();
				}

			}
			break;
 		}

		}

		fobjects.close();
		ftarget.close();
		fltarget.close();


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
	flog.close();
	Aria::shutdown();
	printf("Shutting down");
	return 0;
}
