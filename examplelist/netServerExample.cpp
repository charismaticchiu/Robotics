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

/** @example netServerExample.cpp Shows how to use ArNetServer, a simple text
 * command receiver
 */

// to use the net server, start this program, then telnet to localhost
// 7171, and type in 'password' and press enter, it should then be
// self explanatory

// this function just prints out what the client entered and then sends some 
// data back. You could modify it to, for example, control the robot with
// different commands.
void test(char **argv, int argc, ArSocket *socket)
{
  int i;
  printf("Client said: ");
  for (i = 0; i < argc; ++i)
    printf("\t%s\n", argv[i]);
  printf("\n");
  socket->writeString("Thank you, command received.");
}


int main(int argc, char **argv)
{
  // Initialize Aria
  Aria::init();
  // we need a server
  ArNetServer server;
  // a callback for our test function
  ArGlobalFunctor3<char **, int, ArSocket *> testCB(&test);

  // start the server up without a robot on port 7171 with a password
  // of password and allow multiple clients
  if (!server.open(NULL, 7171, "password", true))
  {
    printf("Could not open server.\n");
    exit(1);
  }
  
  // add our test command
  server.addCommand("test", &testCB, "this simply prints out the command given on the server");
  server.addCommand("test2", &testCB, "this simply prints out the command given on the server");
  
  //server.setLoggingDataSent(true);
  //server.setLoggingDataReceived(true);
  // run while the server is running
  while (server.isOpen() && Aria::getRunning())
  {
    server.runOnce();
    ArUtil::sleep(1);
  }
  server.close();
  return 0;  
}
