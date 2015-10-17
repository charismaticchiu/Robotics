#include "Aria.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <stdio.h>
#include <vector>
#include <math.h>
void turn_left_right(int turn,int distance,float deg,float deg_old,ArRobot* robot);
using namespace std;

#define D_DISTANCE 1000

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
};

tracking_object v_calc(tracking_object objt1, tracking_object objt2, int time)
{
    printf("degree t1 = %f, degree t2 = %f, dt = %i\r\n", objt1.distance, objt2.distance, time);
    objt2.vrad = (objt2.distance - objt1.distance)/(float)time;
    objt2.vtan = (objt2.distance + objt1.distance)*0.5*(objt2.degree-objt1.degree)*(3.1415/180.0)/(float)time;
    objt2.vmag = sqrt((objt2.vtan*objt2.vtan)+(objt2.vrad*objt2.vrad));
    return(objt2);
}

std::vector<tracking_object> run_sick_scan(ArSick * sick)
{
    std::vector<tracking_object> objects1;
    int range1 [361] = {0};
    int drange1 [360] = {0};
    float p_drange1 [360] = {0};
    std::list<ArSensorReading *> *r1;
    std::list<ArSensorReading *>::iterator it;
    
    
    (*sick).lockDevice();
    r1=(list<ArSensorReading *,allocator<ArSensorReading *> > *)(*sick).getRawReadings();
    (*sick).unlockDevice();
    
    
    if(NULL!=r1)
    {
        
        if (r1->end() != r1->begin())
        {
            int i = 0;
            // Process and store data from first reading
            for (it = r1->begin(); it!= r1->end(); it++)
            {
                range1[i] = ((*it)->getRange());
                if(i)
                {
                    drange1[i-1] = range1[i] - range1[i-1];
                    p_drange1[i-1] = (float)drange1[i-1]*2.0/(range1[i]+range1[i-1]);
                    //  printf("%i %i %i\r\n", i, range1[i], drange1[i-1]);
                }
                i++;
            }
            
            i = 10; //ignore the first 5 degrees
            
            int r_edge = 0;
            int l_edge = 0;
            
            // Cycle through first set of readings and identify objects
            while(i < 350) //also ignore the last 5 degrees
            {
                //This if statement looks for the left edge of an object, which shows up as a negative drange
                if(drange1[i] < -500)
                {
                    if(range1[i+1] < 1000)
                    {
                        r_edge = i;
                        while((drange1[i] < 500)&& i < 360)
                        {
                            i++;
                        }
                        l_edge = i;
                        // printf("object: left edge %i\tright edge %i\r\n", l_edge, r_edge);
                        if(i < 360)
                        {
                            tracking_object new_obj;
                            new_obj.double_degree = (r_edge + (l_edge - r_edge)/2);
                            new_obj.degree = new_obj.double_degree/2.0;
                            new_obj.distance = range1[new_obj.double_degree];
                            objects1.push_back(new_obj);
                        }
                    }
                }
                i++;
            }
        }
    }
    return(objects1);
}





std::vector<tracking_object> get_moving_objects(ArSick * sick, int ms_time)
{
    std::vector<tracking_object> objects1;
    std::vector<tracking_object> objects2;
    std::vector<tracking_object> obj_vector;
    
    ArTime start;
    start.setToNow();
    
    objects1 = run_sick_scan(sick);
    usleep(ms_time*1000);
    objects2 = run_sick_scan(sick);
    
    ms_time = start.mSecSince();
    
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
            new_v = v_calc(objects1[i],objects2[j],ms_time);
            
            if (new_v.vmag < v.vmag)
            {
                v = new_v;
            }
        }
        /*
         if(v.vmag > 5.0)
         {
         v.vmag = 0.0;
         v.vrad = 0.0;
         v.vtan = 0.0;
         }
         */
        obj_vector.push_back(v);
        
    }
    /*
     printf("Size of obj_vector is %i\r\n",obj_vector.size());
     for(i = 0; i < obj_vector.size();i++)
     {
     printf("Distance %f\r\n", obj_vector[i].distance);
     printf("Degree %f\r\n", obj_vector[i].degree);
     printf("V mag %f\r\n", obj_vector[i].vmag);
     printf("V rad %f\r\n", obj_vector[i].vrad);
     printf("V tan %f\r\n", obj_vector[i].vtan);
     }
     */
    return(obj_vector);
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
    
    
    /////////////////////////////////////
    
    while(1)
    {
        
        std::vector<tracking_object> obj_vector;
        tracking_object obj_now;
        obj_vector = get_moving_objects(&sick, 100);
        unsigned int i = 0;
        unsigned int num_objects = obj_vector.size();
        unsigned int num_objects_prev = obj_vector.size();
        if(num_objects)
        {
            
            for(i = 0; i < obj_vector.size();i++)
            {
                /*	      
                 printf("Distance %f\r\n", obj_vector[i].distance);
                 printf("Degree %f\r\n", obj_vector[i].degree);
                 printf("V mag %f\r\n", obj_vector[i].vmag);
                 printf("V rad %f\r\n", obj_vector[i].vrad);
                 printf("V tan %f\r\n", obj_vector[i].vtan);
                 */
                if((obj_vector[i].vmag > 0.1))
                {
                    printf("Detected moving object!\r\n");
                    printf("Distance %f\r\n", obj_vector[i].distance);
                    printf("Degree %f\r\n", obj_vector[i].degree);
                    printf("V mag %f\r\n", obj_vector[i].vmag);
                    printf("V rad %f\r\n", obj_vector[i].vrad);
                    printf("V tan %f\r\n", obj_vector[i].vtan);
                    
                    ArUtil::sleep(10);			    
                    robot.lock();
                    robot.setDeltaHeading(obj_vector[i].degree - 90);
                    robot.unlock();
                    ArUtil::sleep(10);
                }
            }     
        }
        obj_now = discern(obj_vector);
        //planning and
        if (deg-deg_old>10)
        {
            turn=1;//left
            if (!in_sight()){//in()sight not declared
                turn_left_right(turn,distance,deg,deg_old,&robot)
                deg_old=deg;
            }else {//if still can see the object, then follow it
                robot.lock();
                robot.setDeltaHeading(deg-deg_old);
                robot.unlock();
            }
        }
        else if (deg-deg_old> -10)
        {
            turn=-1;//right
            if (!in_sight()){//in()sight not declared
                turn_left_right(turn,distance,deg,deg_old,&robot)
                deg_old=deg;
            }else {//if still can see the object, then follow it
                robot.lock();
                robot.setDeltaHeading(deg-deg_old);
                robot.unlock();
            }
        }
        /*
         char dummy;
         printf("scan complete, press any key to repeat.\r\n");
         scanf("%c",&dummy);
         */
    }
    
    Aria::shutdown();
    return 0;
}
tracking_object discern(std::vector<tracking_object> obj_vector){
    for (<#initialization#>; <#condition#>; <#increment#>) {
        <#statements#>
    }


}
bool insight(int num_objects,int num_objects_prev){
    if (num_objects_prev-num_objects=1){
        return true
    }
    else
        false
}
void turn_left_right(int turn,int distance,float deg,float deg_old,ArRobot* robot){
    
    int straight=distance*cos(deg-deg_old);
    robot.lock();
    robot.setVel(2*def_vel);
    robot.unlock();
    ArTime count
    count.setToNow();
    while (1) {
        if (count.mSecSince() > (straight/(2*def_vel)))
            break;
    }
    robot.lock();
    robot.stop();
    robot.unlock();
    ArUril::sleep(50);
    robot.lock();
    robot.setDeltaHeading(turn*90);
    robot.unlock();
    robot.lock();
    robot.setVel(def_vel);
    robot.unlock();
    
    
}
