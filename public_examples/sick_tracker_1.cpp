#include "Aria.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <stdio.h>
#include <math.h>
using namespace std;

#define D_DISTANCE 1000
#define def_vel 50
void turn_left_right(int turn,int distance,float deg,float deg_old,ArRobot* robot)

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



int main(int argc, char **argv)
{
    
    ArRobot robot;
    
    
    Aria::init();
    //laser
    int ret; //Don't know what this variable is for
    ArSick sick; // Laser scanner
    
    ArSerialConnection laserCon; // Scanner connection
    std::string str; // Standard output
    
    //
    
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
        int range [361] = {0};
        int drange [360] = {0};
        int i = 0;
        
        
        std::list<ArSensorReading *> *readings;
        std::list<ArSensorReading *>::iterator it;
        
        sick.lockDevice();
        
        readings=(list<ArSensorReading *,allocator<ArSensorReading *> > *)sick.getRawReadings();
        
        sick.unlockDevice();
        
        if(NULL!=readings)
        {
            printf("Got readings\r\n");
            if ((readings->end() != readings->begin()))
            {
                for (it = readings->begin(); it!= readings->end(); it++)
                {
                    //std::cout << (*it)->getRange()<<" ";
                    
                    range[i] = ((*it)->getRange());
                    
                    if(i)
                    {
                        
                        drange[i-1] = range[i] - range[i-1];
                        printf("%f %i %i\r\n", (float)i/2.0, range[i], drange[i-1]);
                        
                    }
                    i++;
                }
                int i = 0;
                
                if(NULL!=readings)
                {
                    
                    while(i < 360)
                    {
                        int r_edge = 0;
                        int l_edge = 0;
                        int obsticle_range = 0;
                        int obsticle_center_half_deg = 0;
                        float obsticle_degree = 0;
                        //This if statement looks for the left edge of an object, which shows up as a negative drange
                        if(drange[i] > D_DISTANCE)
                            
                        {
                            if(range[i] < 2000)
                            {
                                
                                r_edge = i;
                                while((drange[i] > -D_DISTANCE)&& i < 360)
                                {
                                    i++;
                                }
                                l_edge = i;
                                
                                if(i < 360)
                                {
                                    obsticle_center_half_deg = (r_edge + (l_edge - r_edge)/2);
                                    printf("%i\r\n",obsticle_center_half_deg);
                                    
                                    obsticle_range = range[obsticle_center_half_deg];
                                    
                                    obsticle_degree = obsticle_center_half_deg/2.0;
                                    
                                    printf("\r\n object detected at %f\r\nr edge %i ledge %i", obsticle_degree, r_edge, l_edge);
                                    ArUtil::sleep(100);			    
                                    robot.lock();
                                    
                                    robot.setDeltaHeading(obsticle_degree-90);
                                    //robot.waitForRunExit();
                                    robot.unlock();
                                    ArUtil::sleep(100);
                                    
                                    robot.lock();
                                    robot.setVel(50);
                                    robot.unlock();
                                    /*
                                     robot.lock();
                                     robot.move(obsticle_range);
                                     robot.stop(); 
                                     robot.unlock();
                                     ArUtil::sleep(100);
                                     */
                                    //break;
                                }
                            }
                        }
                        i++;
                    }
                }
                
                
            }	   
            else
            {
                std::cout << "(readings->end() == readings -> begin())" << std::endl;
            }
        }
        
        char dummy;
        printf("scan complete, press any key to repeat.\r\n");
        s       canf("%c",&dummy);
        
        /*********/
        (deg,vel,distance) = tracknig();//deg,vel,distance,deg_old is not declared
        
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
        
        /*********/
        
    }
    
    
    ArUtil::sleep(100);
    robot.lock();
    robot.setDeltaHeading(48);
    robot.unlock();
    
    ArUtil::sleep(3000);
    robot.lock();
    robot.setDeltaHeading(48);
    robot.unlock();
    ArUtil::sleep(3000);
    printf("directMotionExample: Done, shutting down Aria and exiting.\n");
    
    
    Aria::shutdown();
    return 0;
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
float avg_speed(int * obj_range, int * old_range,int * range, float duration ){
    float acc=0.0
    for(int j=obj_range[0];j<obj_range[1]-obj_range[0];j++){
        acc= acc + (range[j]-old_range[j])
    }
    float distance = acc/(obj_range[1]-obj_range[0]);
    float speed= distance/(duration*0.001);//duration is in millisecond
    return speed;
}
