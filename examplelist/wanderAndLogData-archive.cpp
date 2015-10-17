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
#include <string>

/*
 * This is useful as a diagnostic tool, plus it shows all the many accessor
 * methods of ArRobot for robot state. It makes the robot wander, using sonar
 * to avoid obstacles, and prints out various pieces of robot state information
 * each second. Refer to the ARIA ArRobot documentation and to your robot manual
 * (section on standard ARCOS SIP packet contents) for details on the data.
 */



/* function to display a byte as a string of 8 '1' and '0' characters. */
std::string byte_as_bitstring(char byte) 
{
  char tmp[9];
  int bit; 
  int ch;
  for(bit = 7, ch = 0; bit >= 0; bit--,ch++)
    tmp[ch] = ((byte>>bit)&1) ? '1' : '0';
  tmp[8] = 0;
  return std::string(tmp);
}

/* function to display a 2-byte int as a string of 16 '1' and '0' characters. */
std::string int_as_bitstring(ArTypes::Byte2 n) 
{
  char tmp[17];
  int bit;
  int ch;
  for(bit = 15, ch = 0; bit >= 0; bit--, ch++)
    tmp[ch] = ((n>>bit)&1) ? '1' : '0';
  tmp[16] = 0;
  return std::string(tmp);
}

/* Some events might only be detectable in one robot cycle, not over the
 * 1-second period that the main thread sleeps. This cycle callback will detect
 * those and save them in some global variables. */
bool wasLeftMotorStalled = false;
bool wasRightMotorStalled = false;
ArTypes::UByte2 cumulativeStallVal = 0;
ArTypes::UByte2 cumulativeRobotFlags = 0;
bool wasLeftIRTriggered = false;
bool wasRightIRTriggered = false;
bool wasEStopTriggered = false;

bool cycleCallback(ArRobot* robot)
{
  cumulativeStallVal |= robot->getStallValue();
  wasLeftMotorStalled = wasLeftMotorStalled || robot->isLeftMotorStalled();
  wasRightMotorStalled = wasRightMotorStalled || robot->isRightMotorStalled();
  wasEStopTriggered = wasEStopTriggered || robot->getEstop();
  wasLeftIRTriggered = wasLeftIRTriggered || (robot->hasTableSensingIR() && robot->isLeftTableSensingIRTriggered());
  wasRightIRTriggered = wasRightIRTriggered || (robot->hasTableSensingIR() && robot->isRightTableSensingIRTriggered());
  return true;
}


unsigned int encoderPacketCountPrevSec;
unsigned int encoderPacketCount;
ArTime encoderPacketTimer;

bool packetCallback(ArRobotPacket *packet)
{
  if(packet->getID() == 0x90)
  {
    // encoder packet
    ++encoderPacketCount;
    if(encoderPacketTimer.secSince() >= 1)
    {
      encoderPacketCountPrevSec = encoderPacketCount;
      encoderPacketCount = 0;
      encoderPacketTimer.setToNow();
    }
    printf("got an encoder packet. Count=%d, CountPrevSec=%d\n", encoderPacketCount, encoderPacketCountPrevSec);
  }
  return false;  // let other packet handlers (e.g. in ArRobot) be called
}



/* main function */
int main(int argc, char **argv)
{
  // robot and devices
  ArRobot robot;
  ArTcpConnection *connection;
  ArTime time;
  ArSonarDevice sonar;
  ArBumpers bumpers;
  ArIRs ir;
  ArRobotPacket *fmPacket;


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


  // the actions we'll use to wander and avoid obstacles
  ArActionStallRecover recoverAct;
  ArActionBumpers bumpAct;
  ArActionAvoidFront avoidFrontNearAct("Avoid Front Near", 225, 0);
  ArActionAvoidFront avoidFrontFarAct;
  ArActionConstantVelocity constantVelocityAct("Constant Velocity", 400);

  // initialize aria and aria's logging destination and level
  Aria::init();
  ArLog::init(ArLog::StdErr, ArLog::Normal);

  // connector
  ArSimpleConnector connector(&argc, argv);
  if (!connector.parseArgs() || argc > 1)
  {
    connector.logOptions();
    exit(1);
  }

  printf("This program will make the robot wander around, avoiding obstacles, and print some data and events.\nPress Ctrl-C to exit.\n");
  
  // add the range devices to the robot
  robot.addRangeDevice(&sonar);
  robot.addRangeDevice(&bumpers);
  robot.addRangeDevice(&ir);
  
  // try to connect, if we fail exit
  if (!connector.connectRobot(&robot))
  {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    return 1;
  }

  // turn on the motors, turn off amigobot sound effects (for old h8-model amigobots)
  robot.enableMotors();
  robot.comInt(ArCommands::SOUNDTOG, 0);

  // add the actions created above
  robot.addAction(&recoverAct, 100);
  robot.addAction(&bumpAct, 75);
  robot.addAction(&avoidFrontNearAct, 50);
  robot.addAction(&avoidFrontFarAct, 49);
  robot.addAction(&constantVelocityAct, 25);

  // Cycle callback to check for events
  robot.addUserTask("checkevents", 1, new ArGlobalRetFunctor1<bool, ArRobot*>(&cycleCallback, &robot));

  // Packet callback to count packets recieved of different types
  encoderPacketCount = 0;
  encoderPacketCountPrevSec = 0;
  encoderPacketTimer.setToNow();
  robot.addPacketHandler(new ArGlobalRetFunctor1<bool, ArRobotPacket*>(&packetCallback), ArListPos::FIRST);

  // start the robot running, true means that if we lose robot connection the 
  // ArRobot runloop stops
  robot.runAsync(true);
  
  // Request that we will want encoder data
  robot.requestEncoderPackets();

  while(robot.isRunning()) {
    //robot.lock();
    //time_t t = time(NULL);
    //strftime(timestamp, 24, "%Y-%m-%d %H:%M:%S", localtime(&t));
    
    //t = time.getSec();
    //time = connection.getTimeRead(0);

    //gettimeofday(&tv, NULL);
    //timestamp = tv.tv_sec + tv.tv_usec/1000000.0;

  // Lets open the master socket
  if (masterSock.open (portNumber, ArSocket::TCP))
    printf ("Opened the master port at %d\n", portNumber);
  else
    {
      printf ("Failed to open the master port at %d: %s\n",
	      portNumber, masterSock.getErrorStr ().c_str ());
      return -1;
    }


/*  // Lets wait for the client to connect to us.
  if (masterSock.accept (&clientSock))
    printf ("Client has connected\n");
  else
    printf ("Error in accepting a connection from the client: %s\n",
	    masterSock.getErrorStr ().c_str ());*/


  // now set up our connection so our packet receivers work
  clientConn.setSocket (&clientSock);
  clientConn.setStatus (ArDeviceConnection::STATUS_OPEN);
  clientConn.open();
  lastClientTest.setToNow ();
  lastData.setToNow ();


  // while we're open, just read from one port and write to the other
  //while (true) //(clientSock.getFD () >= 0)
    {
      // get our packet
      printf("here again ------------ \n");
	packet = clientRec.receivePacket (1000);
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

robot.lock();

    printf("  %f   %d  \n",robot.getLeftVel(),robot.getOdometryDelay());
    /*printf( DATAFORMAT,
        timestamp,
        robot.getMotorPacCount(),
        robot.getLeftVel(), 
        robot.getRightVel(),
        robot.getLeftEncoder(),
        robot.getRightEncoder(),
        encoderPacketCountPrevSec
      ); */

    robot.unlock();
    ArUtil::sleep(100);
  }
  
  // robot cycle stopped, probably because of lost robot connection
  Aria::shutdown();
  return 0;
}
