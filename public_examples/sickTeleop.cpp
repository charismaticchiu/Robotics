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

/*
  This is an example of how to use the limiting actions.

  It requires a SICK laser. Use either a joystick or keyboard
  to drive the robot.  If the SICK laser detects obstacles, actions
  slow down the robot as it approaches.
  
  The way it works is that it has a limiting behavior higher priority
  than the joydrive and keydrive actions.  So if the limiting behaviors
  detect obstacles that are too close, they set speed limits proportional 
  to the distance that slow down the robot and eventually prevent the 
  joydrive and keydrive actions from having any effect on the robot.
*/


int main(int argc, char **argv)
{
  ArRobot robot;
  ArSick sick;

  ArActionJoydrive jdAct;
  ArActionKeydrive keyAct;
  
  // limiter for close obstacles
  ArActionLimiterForwards limiter("speed limiter near", 300, 600, 250);

  // limiter for far away obstacles
  ArActionLimiterForwards limiterFar("speed limiter far", 300, 1100, 400);

  // limiter that uses the IR sensors on a Peoplebot, if equipped
  ArActionLimiterTableSensor tableLimiter;

  // limiter so we don't bump things backwards
  ArActionLimiterBackwards backwardsLimiter;

  Aria::init();

  robot.addRangeDevice(&sick);

  ArSimpleConnector connector(&argc, argv);
  if (!connector.parseArgs() || argc > 1)
  {
    connector.logOptions();
    exit(1);
  }

  // try to connect, if we fail exit
  if (!connector.connectRobot(&robot))
  {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    return 1;
  }


  // now that we're connected to the robot, connect to the laser
  connector.setupLaser(&sick);
  sick.runAsync();
  if (!sick.blockingConnect())
  {
    printf("Could not connect to SICK laser... exiting\n");
    Aria::shutdown();
    return 1;
  }


  // enable the motors, disable amigobot sounds
  robot.comInt(ArCommands::ENABLE, 1);
  robot.comInt(ArCommands::SOUNDTOG, 0);

  // add the actions, use high priority numbers for the limitier actions.
  robot.addAction(&tableLimiter, 100);
  robot.addAction(&limiter, 95);
  robot.addAction(&limiterFar, 90);
  robot.addAction(&backwardsLimiter, 85);
  robot.addAction(&keyAct, 51);
  robot.addAction(&jdAct, 50);
  
  // run the robot, true means that run() will exit if connection lost
  robot.run(true);
  
  Aria::shutdown();
  return 0;
}
