/**
 * This program is a simple test to see if we
 * can get the sick laser scanner working.
 * Basic initialization code taken from
 * sickWanderUSB.cpp
 */

#define D_DISTANCE 150
#define Default_Distance 200// the direct distance
#define alpha 20 //the edge distance
#include "Aria.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <iomanip>
#include <stdio.h>
#include <time.h>
float avg_speed(int * obj_range, int * old_range,int * range,float duration)
using namespace std;


int main(int argc, char **argv)
{
    int ret; //Don't know what this variable is for
    
    ArRobot robot;// Robot object
    
    ArSick sick; // Laser scanner
    ArSerialConnection laserCon; // Scanner connection
    
    ArSerialConnection con; // Robot connection
    
    std::string str; // Standard output
    
    
    // sonar, must be added to the robot
    ArSonarDevice sonar;
    
    // the actions we'll use to wander
    // recover from stalls
    ArActionStallRecover recover;
    // react to bumpers
    ArActionBumpers bumpers;
    // limiter for close obstacles
    ArActionLimiterForwards limiter("speed limiter near", 300, 600, 250, 1.1);
    // limiter for far away obstacles
    ArActionLimiterForwards limiterFar("speed limiter far", 300, 1100, 600, 1.1);
    // limiter for the table sensors
    ArActionLimiterTableSensor tableLimiter;
    // actually move the robot
    ArActionConstantVelocity constantVelocity("Constant Velocity", 400);
    // turn the orbot if its slowed down
    ArActionTurn turn;
    
    
    
    
    // mandatory init
    Aria::init();
    
    // Parse all our args
    ArSimpleConnector connector(&argc, argv);
    connector.parseArgs();
    
    if (argc > 1)
    {
        connector.logOptions();
        exit(1);
    }
    
    // add the sonar to the robot
    robot.addRangeDevice(&sonar);
    // add the laser to the robot
    robot.addRangeDevice(&sick);
    
    // NOTE: HARDCODED USB PORT!
    // Attempt to open hard-coded USB to robot
    if ((ret = con.open("/dev/ttyUSB2")) != 0){
        // If connection fails, exit
        str = con.getOpenMessage(ret);
        printf("Open failed: %s\n", str.c_str());
        Aria::shutdown();
        return 1;
    }
    
    // set the robot to use the given connection
    robot.setDeviceConnection(&con);
    
    // do a blocking connect, if it fails exit
    if (!robot.blockingConnect())
    {
        printf("Could not connect to robot... exiting\n");
        Aria::shutdown();
        return 1;
    }
    
    
    
    
    // turn on the motors, turn off amigobot sounds
    //robot.comInt(ArCommands::SONAR, 0);
    robot.comInt(ArCommands::SOUNDTOG, 0);
    
    
    
    // start the robot running, true so that if we lose connection the run stops
    robot.runAsync(true);
    
    
    // Attempt to connect to SICK using another hard-coded USB connection
    sick.setDeviceConnection(&laserCon);
    if((ret=laserCon.open("/dev/ttyUSB3")) !=0) {
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
    
    /*
     robot.lock();
     robot.comInt(ArCommands::ENABLE, 1);
     robot.unlock();
     */
    int range [361] = {0};
    int drange [360] = {0};
    int i = 0;
    int obj_range [2];
    int old_range [360]={0};
    clock_t now, prev;
    while(1){
        range [361] = {0};
        drange [360] = {0};
        i = 0;
        obj_range[2];
        
        std::list<ArSensorReading *> *readings;
        std::list<ArSensorReading *>::iterator it;
        sick.lockDevice();
        readings=(list<ArSensorReading *,allocator<ArSensorReading *> > *)sick.getRawReadings();
        
        if(NULL!=readings){
            if ((readings->end() != readings->begin())){
                for (it = readings->begin(); it!= readings->end(); it++){
                    //	      std::cout << (*it)->getRange()<<" ";
                    range[i] = ((*it)->getRange());
                    if(i){
                        drange[i-1] = range[i] - range[i-1];
                        printf("%f %i %i\r\n", (float)i/2.0, range[i], drange[i-1]);
                    }
                    i++;
                }
                int i = 0;
                //detect the object range
                while (i < 360) {
                    if (range[i]>Default_Distance + alpha) {
                        ;
                    } else {
                        if (obj_range[0]=0)
                            obj_range[0]=i;
                        else
                            obj_range[1]=i;
                    }
                }
                if (!now)
                    prev=now;
                now=clock();
                duration=now-prev;
                /******moving straight*******/
                float speed = avg_speed(obj_range,old_range,range,(float)duration)
                
                /*while(i < 360){
                 int r_edge = 0;
                 int l_edge = 0;
                 float obsticle_degree = 0;
                 if(drange[i] > D_DISTANCE){
                 r_edge = i;
                 while(drange[i] > -(D_DISTANCE)){
                 i++;
                 }
                 l_edge = i;
                 obsticle_degree = (r_edge + (l_edge - r_edge)/2.0)/2.0;
                 printf("\r\n object detected at %f\r\n", obsticle_degree);
                 }
                 std::cout<<std::endl;
                 }*/
            }
            else{
                std::cout << "(readings->end() == readings -> begin())" << std::endl;
            }
        }
        else{
            std::cout << "NULL == readings" << std::endl;
        }
        
        sick.unlockDevice();
        /*char dummy;
         printf("Scan complete. Hit any key to run another scan.");
         scanf("%c", &dummy);
         */
    }
    
    robot.waitForRunExit();
    // now exit
    Aria::shutdown();
    return 0;
}


float avg_speed(int * obj_range, int * old_range,int * range, float duration){
    float acc=0.0
    for(int j=obj_range[0];j<obj_range[1]-obj_range[0];j++){
        acc= acc + (range[j]-old_range[j])
    }
    float distance = acc/(obj_range[1]-obj_range[0]);
    float speed= distance/(duration*0.001);//duration is in millisecond
    return speed;
    
}
