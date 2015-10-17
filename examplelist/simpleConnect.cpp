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

/** @example simpleConnect.cpp example showing how to connect to the robot with ArRobotConnector
 *
 * One of the simplest ARIA programs possible:
 * Connects with ArRobotConnector, waits 3 seconds doing
 * nothing, then exits.
 *
 * This program will work either with the MobileSim simulator or on a real
 * robot's onboard computer.  (Or use -remoteHost to connect to a wireless
 * ethernet-serial bridge.)
 */

int
main (int argc, char **argv)
{
  Aria::init ();
  ArRobot robot;
  ArArgumentParser parser (&argc, argv);
  parser.loadDefaultArguments ();

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
  const char *portName;

  // Connect to the robot, get some initial data from it such as type and name,
  // and then load parameter files for this robot.
  ArRobotConnector robotConnector (&parser, &robot);
  if (!robotConnector.connectRobot ())
    {
      ArLog::log (ArLog::Terse,
		  "simpleConnect: Could not connect to the robot.");
      if (parser.checkHelpAndWarnUnparsed ())
	{
	  // -help not given
	  Aria::logOptions ();
	  Aria::exit (1);
	}
    }
  if (!Aria::parseArgs ())
    {
      Aria::logOptions ();
      Aria::shutdown ();
      return 1;
    }

  ArLog::log (ArLog::Normal, "simpleConnect: Connected.");

  // Start the robot processing cycle running in the background.
  // True parameter means that if the connection is lost, then the 
  // run loop ends.
  robot.runAsync (true);

  // Print out some data from the SIP.  We must "lock" the ArRobot object
  // before calling its methods, and "unlock" when done, to prevent conflicts
  // with the background thread started by the call to robot.runAsync() above.
  // See the section on threading in the manual for more about this.
  robot.lock ();
  ArLog::log (ArLog::Normal,
	      "simpleConnect: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Battery=%.2fV",
	      robot.getX (), robot.getY (), robot.getTh (), robot.getVel (),
	      robot.getBatteryVoltage ());
  robot.unlock ();

  // Sleep for 3 seconds.
  ArLog::log (ArLog::Normal, "simpleConnect: Sleeping for 3 seconds...");
  ArUtil::sleep (300);


  //------------------------------------------------------------------------------------------
  robot.enableMotors ();
  robot.requestEncoderPackets ();

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







  ArLog::log (ArLog::Normal, "simpleConnect: Ending robot thread...");
  robot.stopRunning ();

  // wait for the thread to stop
  robot.waitForRunExit ();

  // exit
  ArLog::log (ArLog::Normal, "simpleConnect: Exiting.");
  return 0;
}
