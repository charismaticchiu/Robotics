/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 10 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/
#include "Aria.h"

unsigned int encoderPacketCountPrevSec;
unsigned int encoderPacketCount;
ArTime encoderPacketTimer;

/** @example demo.cpp General purpose testing and demo program, using ArMode
 *    classes to provide keyboard control of various robot functions.
 *
 *  demo uses ArMode subclasses, which are pre-made classes in ARIA that 
 *  provide keyboard control of various aspects and accessories of 
 *  the robot.  The ArMode classes are defined in %ArModes.cpp.
 *
 *  "demo" is a useful program for testing out the operation of the robot
 *  for diagnostic or demonstration purposes.  Other example programs
 *  focus on individual areas.
 */

int main(int argc, char** argv)
{
  // Initialize some global data
  Aria::init();

  // If you want ArLog to print "Verbose" level messages uncomment this:
  //ArLog::init(ArLog::StdOut, ArLog::Verbose);

  // This object parses program options from the command line
  ArArgumentParser parser(&argc, argv);

  // Load some default values for command line arguments from /etc/Aria.args
  // (Linux) or the ARIAARGS environment variable.
  parser.loadDefaultArguments();

  // Central object that is an interface to the robot and its integrated
  // devices, and which manages control of the robot by the rest of the program.
  ArRobot robot;

  // Object that connects to the robot or simulator using program options
  ArRobotConnector robotConnector(&parser, &robot);

  // If the robot has an Analog Gyro, this object will activate it, and 
  // if the robot does not automatically use the gyro to correct heading,
  // this object reads data from it and corrects the pose in ArRobot
  ArAnalogGyro gyro(&robot);

  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  if (!robotConnector.connectRobot())
  {
    // Error connecting:
    // if the user gave the -help argumentp, then just print out what happened,
    // and continue so options can be displayed later.
    if (!parser.checkHelpAndWarnUnparsed())
    {
      ArLog::log(ArLog::Terse, "Could not connect to robot, will not have parameter file so options displayed later may not include everything");
    }
    // otherwise abort
    else
    {
      ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
      Aria::logOptions();
      Aria::exit(1);
    }
  }

  // Connector for laser rangefinders
  ArLaserConnector laserConnector(&parser, &robot, &robotConnector);

  // Connector for compasses
  ArCompassConnector compassConnector(&parser);

  // Parse the command line options. Fail and print the help message if the parsing fails
  // or if the help was requested with the -help option
  if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {    
    Aria::logOptions();
    exit(1);
  }

  // Used to access and process sonar range data
  ArSonarDevice sonarDev;
  
  // Used to perform actions when keyboard keys are pressed
  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);

  // ArRobot contains an exit action for the Escape key. It also 
  // stores a pointer to the keyhandler so that other parts of the program can
  // use the same keyhandler.
  robot.attachKeyHandler(&keyHandler);
  printf("You may press escape to exit\n");

  // Attach sonarDev to the robot so it gets data from it.
  robot.addRangeDevice(&sonarDev);

  // Cycle callback to check for events
  //robot.addUserTask("checkevents", 1, new ArGlobalRetFunctor1<bool, ArRobot*>(&cycleCallback, &robot));

  // Packet callback to count packets recieved of different types
  encoderPacketCount = 0;
  encoderPacketCountPrevSec = 0;
  encoderPacketTimer.setToNow();
  //robot.addPacketHandler(new ArGlobalRetFunctor1<bool, ArRobotPacket*>(&packetCallback), ArListPos::FIRST);
  
  // Start the robot task loop running in a new background thread. The 'true' argument means if it loses
  // connection the task loop stops and the thread exits.
  robot.runAsync(true);


  robot.requestEncoderPackets();

    struct timeval tv;
    double timestamp;
    long LastOdomMSec=1000;

    ArTime OdomTime;
    ArPose RawPose;
    const ArRobotParams *RobParam;

    RobParam = robot.getRobotParams();

    printf("AngleConvFactor: %f \nDiffConvFactor: %f \nDistConvFactor: %f \nVelConvFactor: %f \n",
		    RobParam->getAngleConvFactor(),
		    RobParam->getDiffConvFactor(),
		    RobParam->getDistConvFactor(),
		    RobParam->getVelConvFactor());

    FILE *file = fopen("pioneer.dat","w"); 
    if (file == NULL)
	    return -1;


  // Sleep for a second so some messages from the initial responses
  // from robots and cameras and such can catch up
  ArUtil::sleep(1000);
  // Connect to the laser(s) if lasers were configured in this robot's parameter
  // file or on the command line, and run laser processing thread if applicable
  // for that laser class.  (For the purposes of this demo, add all
  // possible lasers to ArRobot's list rather than just the ones that were
  // specified with the connectLaser option (so when you enter laser mode, you
  // can then interactively choose which laser to use from the list which will
  // show both connected and unconnected lasers.)
  if (!laserConnector.connectLasers(false, false, true))
  {
    printf("Could not connect to lasers... exiting\n");
    Aria::exit(2);
  }

  // Create and connect to the compass if the robot has one.
  ArTCM2 *compass = compassConnector.create(&robot);
  if(compass && !compass->blockingConnect()) {
    compass = NULL;
  }
  
  // Sleep for a second so some messages from the initial responses
  // from robots and cameras and such can catch up
  ArUtil::sleep(1000);

  // We need to lock the robot since we'll be setting up these modes
  // while the robot task loop thread is already running, and they 
  // need to access some shared data in ArRobot.
  robot.lock();

  // now add all the modes for this demo
  // these classes are defined in ArModes.cpp in ARIA's source code.
  ArModeLaser laser(&robot, "laser", 'l', 'L');
  ArModeTeleop teleop(&robot, "teleop", 't', 'T');
  ArModeUnguardedTeleop unguardedTeleop(&robot, "unguarded teleop", 'u', 'U');
  ArModeWander wander(&robot, "wander", 'w', 'W');
  ArModeGripper gripper(&robot, "gripper", 'g', 'G');
  ArModeCamera camera(&robot, "camera", 'c', 'C');
  ArModeSonar sonar(&robot, "sonar", 's', 'S');
  ArModeBumps bumps(&robot, "bumps", 'b', 'B');
  ArModePosition position(&robot, "position", 'p', 'P', &gyro);
  ArModeIO io(&robot, "io", 'i', 'I');
  ArModeActs actsMode(&robot, "acts", 'a', 'A');
  ArModeCommand command(&robot, "command", 'd', 'D');
  ArModeTCM2 tcm2(&robot, "tcm2", 'm', 'M', compass);


  // activate the default mode
  teleop.activate();

  // turn on the motors
  robot.comInt(ArCommands::ENABLE, 1);

  robot.unlock();
  
  while(1) {

    robot.lock();

    gettimeofday(&tv, NULL);
    timestamp = tv.tv_sec + tv.tv_usec/1000000.0;

    OdomTime = robot.getLastOdometryTime();

    if (OdomTime.mSecSince() < LastOdomMSec) {

    RawPose = robot.getRawEncoderPose();
    
    fprintf(file,"%.6f %ld %d %d %f %f %ld %ld %f %f %f %f %f %f %f %f %f %d \n",
        timestamp,
	OdomTime.mSecSince(),
	robot.getOdometryDelay(),
        robot.getMotorPacCount(),
        robot.getLeftVel(), 
        robot.getRightVel(),
        robot.getLeftEncoder(),
        robot.getRightEncoder(),
	robot.getRotVel(),
	robot.getVel(),
	robot.getOdometerDegrees(),
	robot.getTh(),
	robot.getX(),
	robot.getY(),
	RawPose.getTh(),
	RawPose.getX(),
	RawPose.getY(),
        encoderPacketCountPrevSec
      );

    }

    LastOdomMSec = OdomTime.mSecSince();

    robot.unlock();
    ArUtil::sleep(10);
  }
  // Block execution of the main thread here and wait for the robot's task loop
  // thread to exit (e.g. by robot disconnecting, escape key pressed, or OS
  // signal)
  //robot.waitForRunExit();

  Aria::exit(0);


}

