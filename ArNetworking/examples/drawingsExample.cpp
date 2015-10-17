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
#include "ArNetworking.h"
#include <math.h>


/** @example drawingsExample.cpp Example showing how to draw custom graphics in
 * clients such as MobileEyes
 *
 * This is an example server that shows how to draw arbitrary figures in a
 * client (e.g. MobileEyes) via ArServerInfoDrawings.  It draws some lines, "arrows", and moving dots
 * with various sizes and colors.   You can use these drawings for debugging
 * or visualization, for example, to represent sensor readings. In fact,
 * specific support for ArRobot, ArSick, ArSonarDevice, ArIRs and ArBumpers
 * are built in to ArServerInfoDrawings: see drawingsExampleWithRobot.cpp
 * or serverDemo.cpp.
 *
 * @sa drawingsExampleWithRobot.cpp
 */


/* These are callbacks that respond to client requests for the drawings' 
 * geometry data. */
void exampleHomeDrawingNetCallback(ArServerClient* client, ArNetPacket* requestPkt);
void exampleDotsDrawingNetCallback(ArServerClient* client, ArNetPacket* requestPkt);
void exampleXDrawingNetCallback(ArServerClient* client, ArNetPacket* requestPkt);
void exampleArrowsDrawingNetCallback(ArServerClient* client, ArNetPacket* requestPkt);

int main(int argc, char **argv)
{
  Aria::init();
  ArServerBase server;

  ArArgumentParser parser(&argc, argv);
  ArServerSimpleOpener simpleOpener(&parser);

  // parse the command line... fail and print the help if the parsing fails
  // or if help was requested
  parser.loadDefaultArguments();
  if (!simpleOpener.parseArgs() || !parser.checkHelpAndWarnUnparsed())
  {    
    simpleOpener.logOptions();
    exit(1);
  }


  // first open the server up
  if (!simpleOpener.open(&server))
  {
    if (simpleOpener.wasUserFileBad())
      printf("Error: Bad user/password/permissions file.\n");
    else
      printf("Error: Could not open server port. Use -help to see options.\n");
    exit(1);
  }


  // This is the service that provides drawing data to the client.
  ArServerInfoDrawings drawings(&server);

  // Add our custom drawings
  drawings.addDrawing(
      //                shape:      color:               size:   layer:
      new ArDrawingData("polyLine", ArColor(255, 0, 0),  2,      49),
      "exampleDrawing_Home", 
      new ArGlobalFunctor2<ArServerClient*, ArNetPacket*>(&exampleHomeDrawingNetCallback)
  );
  drawings.addDrawing(                                    
      new ArDrawingData("polyDots", ArColor(0, 255, 0), 250, 48),
      "exampleDrawing_Dots", 
      new ArGlobalFunctor2<ArServerClient*, ArNetPacket*>(&exampleDotsDrawingNetCallback)
  );
  drawings.addDrawing(
      new ArDrawingData("polySegments", ArColor(0, 0, 0), 4, 52),
      "exampleDrawing_XMarksTheSpot", 
      new ArGlobalFunctor2<ArServerClient*, ArNetPacket*>(&exampleXDrawingNetCallback)
  );
  drawings.addDrawing(
      new ArDrawingData("polyArrows", ArColor(255, 0, 255), 500, 100),
      "exampleDrawing_Arrows", 
      new ArGlobalFunctor2<ArServerClient*, ArNetPacket*>(&exampleArrowsDrawingNetCallback)
  );


  // log whatever we wanted to before the runAsync
  simpleOpener.checkAndLog();

  // run the server thread in the background
  server.runAsync();

  printf("Server is now running...\n");


  // Add a key handler mostly that windows can exit by pressing
  // escape, note that the key handler prevents you from running this program
  // in the background on Linux.
  ArKeyHandler *keyHandler;
  if ((keyHandler = Aria::getKeyHandler()) == NULL)
  {
    keyHandler = new ArKeyHandler;
    Aria::setKeyHandler(keyHandler);
    printf("To exit, press escape.\n");
  }

 
  // run forever
  while(true) ArUtil::sleep(10000);

  Aria::shutdown();
  exit(0);  
}




// Network callbacks for drawings' current geometry data:

void exampleHomeDrawingNetCallback(ArServerClient* client, ArNetPacket* requestPkt) {
  ArNetPacket reply;

  // 7 Vertices
  reply.byte4ToBuf(7);

  // Centered on 0,0.
  // X:                    Y:
  reply.byte4ToBuf(-500);  reply.byte4ToBuf(500);   // Vertex 1
  reply.byte4ToBuf(-500);  reply.byte4ToBuf(-500);  // Vertex 2
  reply.byte4ToBuf(500);   reply.byte4ToBuf(-500);  // Vertex 3
  reply.byte4ToBuf(500);   reply.byte4ToBuf(500);   // Vertex 4
  reply.byte4ToBuf(0);     reply.byte4ToBuf(1000);  // Vertex 5
  reply.byte4ToBuf(-500);  reply.byte4ToBuf(500);   // Vertex 6
  reply.byte4ToBuf(500);   reply.byte4ToBuf(500);   // Vertex 7

  client->sendPacketUdp(&reply);
}

void exampleDotsDrawingNetCallback(ArServerClient* client, ArNetPacket* requestPkt) {
  ArNetPacket reply;

  unsigned int tik = ArUtil::getTime() % 200;
  double t = tik / 5.0;

  // Three dots
  reply.byte4ToBuf(3);

  // Dot 1:
  reply.byte4ToBuf(3000);  // X coordinate (mm)
  reply.byte4ToBuf((int) (sin(t) * 1000));// Y

  // Dot 2:
  reply.byte4ToBuf(3500);  // X
  reply.byte4ToBuf((int) (sin(t+500) * 1000));// Y

  // Dot 3:
  reply.byte4ToBuf(4000);  // X
  reply.byte4ToBuf((int) (sin(t+1000) * 1000));// Y

  client->sendPacketUdp(&reply);
}

void exampleXDrawingNetCallback(ArServerClient* client, ArNetPacket* requestPkt) {
  ArNetPacket reply;

  // X marks the spot. 2 line segments, so 4 vertices:
  reply.byte4ToBuf(4);

  // Segment 1:
  reply.byte4ToBuf(-4250); // X1
  reply.byte4ToBuf(250);   // Y1
  reply.byte4ToBuf(-3750); // X2
  reply.byte4ToBuf(-250);  // Y2

  // Segment 2:
  reply.byte4ToBuf(-4250); // X1
  reply.byte4ToBuf(-250);  // Y1
  reply.byte4ToBuf(-3750); // X2
  reply.byte4ToBuf(250);   // Y2
  
  client->sendPacketUdp(&reply);
}

void exampleArrowsDrawingNetCallback(ArServerClient* client, ArNetPacket* requestPkt) {
  // 1 Arrow that points at the robot
  ArNetPacket reply;
  reply.byte4ToBuf(1);
  reply.byte4ToBuf(0);      // Pos. X
  reply.byte4ToBuf(700);   // Pos. Y
  client->sendPacketUdp(&reply);
}

