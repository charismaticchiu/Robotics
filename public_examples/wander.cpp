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

/** @example wander.cpp 
 * Example using actions and range devices to implement a random wander avoiding obstacles.
 *
 *  This program will just have the robot wander around. It uses some avoidance 
 *  actions if obstacles are detected with the sonar or laser (if robot has a
 *  laser), otherwise it just has a constant forward velocity.
 * 
 *  Press Control-C or Escape keys to exit.
 *  
 * This program will work either with the MobileSim simulator or on a real
 * robot's onboard computer.  (Or use -remoteHost to connect to a wireless
 * ethernet-serial bridge.)
*/

int main(int argc, char **argv)
{
  Aria::init();
  ArArgumentParser argParser(&argc, argv);
  argParser.loadDefaultArguments();
  ArRobot robot;
  ArRobotConnector robotConnector(&argParser, &robot);
  ArLaserConnector laserConnector(&argParser, &robot, &robotConnector);

  int timeout = 0;
  bool tracePackets = true;

  // The socket objects
  ArSocket masterSock, clientSock;
  // The connections
  ArTcpConnection clientConn;
  //ArSerialConnection robotConn;
  // the receivers, first for the robot
  ArRobotPacketReceiver clientRec (&clientConn);
  //ArRobotPacketReceiver robotRec(&robotConn);
  // how about a packet
  ArBasePacket *packet;
  // our timer for how often we test the client
  ArTime lastClientTest;
  ArTime lastData;
  // where we're forwarding from and to
  int portNumber = 8101;

  // Always try to connect to the first laser:
  argParser.addDefaultArgument("-connectLaser");

  if(!robotConnector.connectRobot())
  {
    ArLog::log(ArLog::Terse, "Could not connect to the robot.");
    if(argParser.checkHelpAndWarnUnparsed())
    {
        // -help not given, just exit.
        Aria::logOptions();
        Aria::exit(1);
    }
  }


  // Trigger argument parsing
  if (!Aria::parseArgs() || !argParser.checkHelpAndWarnUnparsed())
  {
    Aria::logOptions();
    Aria::exit(1);
  }

  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);
  robot.attachKeyHandler(&keyHandler);

  puts("This program will make the robot wander around. It uses some avoidance\n"
  "actions if obstacles are detected, otherwise it just has a\n"
  "constant forward velocity.\n\nPress CTRL-C or Escape to exit.");
  
  ArSonarDevice sonar;
  robot.addRangeDevice(&sonar);

  robot.runAsync(true);

  
  // try to connect to laser. if fail, warn but continue, using sonar only
  if(!laserConnector.connectLasers())
  {
    ArLog::log(ArLog::Normal, "Warning: unable to connect to requested lasers, will wander using robot sonar only.");
  }
  
  ArLog::init(ArLog::StdErr, ArLog::Normal);


  // turn on the motors, turn off amigobot sounds
  robot.enableMotors();
  robot.comInt(ArCommands::SOUNDTOG, 0);

  // add a set of actions that combine together to effect the wander behavior
  ArActionStallRecover recover;
  ArActionBumpers bumpers;
  ArActionAvoidFront avoidFrontNear("Avoid Front Near", 225, 0);
  ArActionAvoidFront avoidFrontFar;
  ArActionConstantVelocity constantVelocity("Constant Velocity", 400);
  robot.addAction(&recover, 100);
  robot.addAction(&bumpers, 75);
  robot.addAction(&avoidFrontNear, 50);
  robot.addAction(&avoidFrontFar, 49);
  printf("here.........\n");
  robot.addAction(&constantVelocity, 25);


  // Lets open the master socket
  if (masterSock.open (portNumber, ArSocket::TCP))
    printf ("Opened the master port at %d\n", portNumber);
  else
    {
      printf ("Failed to open the master port at %d: %s\n",
	      portNumber, masterSock.getErrorStr ().c_str ());
      return -1;
    }


  // Lets wait for the client to connect to us.
  if (masterSock.accept (&clientSock))
    printf ("Client has connected\n");
  else
    printf ("Error in accepting a connection from the client: %s\n",
	    masterSock.getErrorStr ().c_str ());


  // now set up our connection so our packet receivers work
  clientConn.setSocket (&clientSock);
  clientConn.setStatus (ArDeviceConnection::STATUS_OPEN);
  lastClientTest.setToNow ();
  lastData.setToNow ();


  // while we're open, just read from one port and write to the other
  while (clientSock.getFD () >= 0)
    {
      // get our packet
      if (true)
	packet = clientRec.receivePacket (1);
      // see if we had one
      if (packet != NULL)
	{
	  if (tracePackets)
	    {
	      printf ("Client ");
	      packet->log ();
	    }
	  lastData.setToNow ();
	}


      ArUtil::sleep (1);
      // If no datas gone by in timeout ms assume our connection is broken
      if (lastData.mSecSince () > timeout)
	{
	  printf
	    ("No data received in %d milliseconds, closing connection.\n",
	     timeout);
	  clientConn.close ();
	}
    }
  // Now lets close the connection to the client
  clientConn.close ();
  printf ("Socket to client closed\n");

  // And lets close the master port
  masterSock.close ();
  printf ("Master socket closed and program exiting\n");

  
  // wait for robot task loop to end before exiting the program
  robot.waitForRunExit();
  
  Aria::exit(0);
}
