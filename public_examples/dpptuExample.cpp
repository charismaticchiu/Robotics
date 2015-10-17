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

/** @example dpptuExample.cpp  Shows how to control the Directed Perceptions Pan-Tilt Unit.
 *
 * This program lets you use the keyboard to control the DPPTU.  It uses the same acceleration and slew rates for the pan and tilt axes.

Commands:
_________________

UP,DOWN  -- tilt up/down by one positional increment
LEFT,RIGHT  --  pan left/right by one positional increment
SPACE  -- perform reset calibration
I  -- initialize PTU to default settings
<,>  -- increase/decrease the posIncrement by 1 degree
+,-  -- increase/decrease the speed by 1 degree/sec
A  -- awaits the completion of last issued positional command
R  -- change pan/tilt movements to relative or absolute movements
Z  -- move pan and tilt axes to zero
M  -- Enter or Exit monitor mode
H  -- Halt all motion
S  -- print current variable values
ESC -- quit
*/

// If defined, use this computer serial port. If not defined, use first robot
// aux. serial port.  Most robots have the DPPTU on COM2 if on Linux and COM4 on
// Windows, if not equipped with other accessories which might require those
// ports (e.g. GPS or Laser).
#define SERIAL_PORT ArUtil::COM4

/*
  This class is the core of this demo, it adds itself to the robot given
  as a user task, and contains key handler callbacks to control the PTU.
*/
class KeyPTU
{
public:
  // constructor
  KeyPTU(ArRobot *robot);
  ~KeyPTU(void);
  
  void up(void);
  void down(void);
  void left(void);
  void right(void);
  void space(void);
  void i(void);
  void plus(void);
  void minus(void);
  void greater(void);
  void less(void);
  void question(void);
  void status(void);
  void a(void);
  void z(void);
  void m(void);
  void h(void);
  void r(void);

  // the callback function
  void drive(void);

protected:
  int myPanValPTU;
  int myTiltValPTU;

  int myDesiredPanPos;
  int myDesiredTiltPos;
  int mySlew;
  int myPosIncrement;
  int mySlewIncrement;

  int posIncIncrement;

  bool myMonitor;
  bool myReset;
  bool myInit;
  bool myAbsolute;

  ArFunctorC<KeyPTU> myUpCB;
  ArFunctorC<KeyPTU> myDownCB;
  ArFunctorC<KeyPTU> myLeftCB;
  ArFunctorC<KeyPTU> myRightCB;
  ArFunctorC<KeyPTU> mySpaceCB;
  ArFunctorC<KeyPTU> myICB;
  ArFunctorC<KeyPTU> myPlusCB;
  ArFunctorC<KeyPTU> myMinusCB;
  ArFunctorC<KeyPTU> myGreaterCB;
  ArFunctorC<KeyPTU> myLessCB;
  ArFunctorC<KeyPTU> myQuestionCB;
  ArFunctorC<KeyPTU> mySCB;
  ArFunctorC<KeyPTU> myACB;
  ArFunctorC<KeyPTU> myZCB;
  ArFunctorC<KeyPTU> myMCB;
  ArFunctorC<KeyPTU> myHCB;
  ArFunctorC<KeyPTU> myRCB;


  // the PTU
  ArDPPTU myPTU;
  

  // whether the PTU has been inited or not
  bool myPTUInited;
  // pointer to the robot
  ArRobot *myRobot;
  // callback for the drive function
  ArFunctorC<KeyPTU> myDriveCB;

  ArSerialConnection *mySerialConnection;
};

/*
  Constructor, sets the robot pointer, and some initial values, also note the
  use of constructor chaining on myPTU and myDriveCB.
*/
KeyPTU::KeyPTU(ArRobot *robot) :
  myUpCB(this, &KeyPTU::up),
  myDownCB(this, &KeyPTU::down),
  myLeftCB(this, &KeyPTU::left),
  myRightCB(this, &KeyPTU::right),
  mySpaceCB(this, &KeyPTU::space),
  myICB(this, &KeyPTU::i),
  myPlusCB(this, &KeyPTU::plus),
  myMinusCB(this, &KeyPTU::minus),
  myGreaterCB(this, &KeyPTU::greater),
  myLessCB(this, &KeyPTU::less),
  myQuestionCB(this, &KeyPTU::question),
  mySCB(this, &KeyPTU::status),
  myACB(this, &KeyPTU::a),
  myZCB(this, &KeyPTU::z),
  myMCB(this, &KeyPTU::m),
  myHCB(this, &KeyPTU::h),
  myRCB(this, &KeyPTU::r),
  myPTU(robot),
  myDriveCB(this, &KeyPTU::drive),
  mySerialConnection(NULL)
{
#ifdef SERIAL_PORT
  mySerialConnection = new ArSerialConnection;
  ArLog::log(ArLog::Normal, "dpptuExample: connecting to DPPTU over computer serial port %s.", SERIAL_PORT);
  if(mySerialConnection->open(SERIAL_PORT) != 0)
  {
	ArLog::log(ArLog::Terse, "dpptuExample: Error: Could not open computer serial port %s for DPPTU!", SERIAL_PORT);
    Aria::exit(5);
  }
  myPTU.setDeviceConnection(mySerialConnection);
#endif

  // set the robot pointer and add the KeyPTU as user task
  ArKeyHandler *keyHandler;
  myRobot = robot;
  myRobot->addSensorInterpTask("KeyPTU", 50, &myDriveCB);

  if ((keyHandler = Aria::getKeyHandler()) == NULL)
  {
    keyHandler = new ArKeyHandler;
    Aria::setKeyHandler(keyHandler);
    myRobot->attachKeyHandler(keyHandler);
  }

  if (!keyHandler->addKeyHandler(ArKeyHandler::UP, &myUpCB))
    ArLog::log(ArLog::Terse, "The key handler already has a key for up, keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler(ArKeyHandler::DOWN, &myDownCB))
    ArLog::log(ArLog::Terse, "The key handler already has a key for down, keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler(ArKeyHandler::LEFT, &myLeftCB))
    ArLog::log(ArLog::Terse,  
"The key handler already has a key for left, keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler(ArKeyHandler::RIGHT, &myRightCB))
    ArLog::log(ArLog::Terse,  
"The key handler already has a key for right, keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler(ArKeyHandler::SPACE, &mySpaceCB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for space, keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler('i', &myICB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for 'i', keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler('+', &myPlusCB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for '+', keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler('-', &myMinusCB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for '-', keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler('>', &myGreaterCB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for '>', keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler('<', &myLessCB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for '<', keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler('?', &myQuestionCB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for '?', keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler('s', &mySCB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for 'S', keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler('a', &myACB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for 'A', keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler('z', &myZCB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for 'Z', keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler('m', &myMCB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for 'M', keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler('h', &myHCB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for 'H', keydrive will not work correctly.");
  if (!keyHandler->addKeyHandler('r', &myRCB))
    ArLog::log(ArLog::Terse,
"The key handler already has a key for 'R', keydrive will not work correctly.");

  // initialize some variables
  myReset = false;
  myInit = true;
  myDesiredPanPos = 0;
  myDesiredTiltPos = 0;
  myPosIncrement = 1;
  mySlewIncrement = 1;
  myPTUInited = false;
  posIncIncrement = 1;
  myMonitor = false;

}

KeyPTU::~KeyPTU() 
{
  if(mySerialConnection)
  {
    myPTU.setDeviceConnection(NULL);
    delete mySerialConnection;
  }
}



void KeyPTU::left(void)
{
  myDesiredPanPos += myPosIncrement;

  if (myDesiredPanPos > myPTU.getMaxPosPan())
    myDesiredPanPos = myPTU.getMaxPosPan();
}

void KeyPTU::right(void)
{
  myDesiredPanPos -= myPosIncrement;

  if (myDesiredPanPos < myPTU.getMaxNegPan())
    myDesiredPanPos = myPTU.getMaxNegPan();
}

void KeyPTU::up(void)
{
  myDesiredTiltPos += myPosIncrement;
 
  if (myDesiredTiltPos > myPTU.getMaxPosTilt())
    myDesiredTiltPos = myPTU.getMaxPosTilt();
}

void KeyPTU::down(void)
{
  myDesiredTiltPos -= myPosIncrement;

  if (myDesiredTiltPos < myPTU.getMaxNegTilt())
    myDesiredTiltPos = myPTU.getMaxNegTilt();
}

void KeyPTU::space(void)
{
  myReset = true;
}

void KeyPTU::i(void)
{
  myInit = true;
}

void KeyPTU::plus(void)
{
  mySlew += mySlewIncrement;

  if (mySlew > myPTU.getMaxPanSlew())
    mySlew = myPTU.getMaxPanSlew();

  status();
}

void KeyPTU::minus(void)
{
  mySlew -= mySlewIncrement;

  if (mySlew < myPTU.getMinPanSlew())
    mySlew = myPTU.getMinPanSlew();

  status();
}

void KeyPTU::greater(void)
{
  myPosIncrement += posIncIncrement;
  
  if (myPosIncrement > myPTU.getMaxPosPan())
    myPosIncrement = myPTU.getMaxPosPan();

  status();
}

void KeyPTU::less(void)
{
  myPosIncrement -= posIncIncrement;

  if (myPosIncrement < 0)
    myPosIncrement = 0;

  status();
}

void KeyPTU::a(void)
{
  myPTU.awaitExec();   
  ArLog::log(ArLog::Normal, "AwaitExecution command sent");
}

void KeyPTU::z(void)
{
  myPTU.pan(0);
  myPTU.awaitExec();
  myPTU.tilt(0);
  myPTU.awaitExec();
  myDesiredPanPos = 0;
  myDesiredTiltPos = 0;
  status();
}

void KeyPTU::question(void)
{
  ArLog::log(ArLog::Normal, "\r\nCommands:\r\n_________________\r\n");
  ArLog::log(ArLog::Normal, "UP,DOWN    -- tilt up/down by one positional increment");
  ArLog::log(ArLog::Normal, "LEFT,RIGHT -- pan left/right by one positional increment");
  ArLog::log(ArLog::Normal, "SPACE      -- perform reset calibration");
  ArLog::log(ArLog::Normal, "I          -- initialize PTU to default settings");
  ArLog::log(ArLog::Normal, "<,>        -- increase/decrease the posIncrement by 1 degree");
  ArLog::log(ArLog::Normal, "+,-        -- increase/decrease the speed by 1 degree/sec");
  ArLog::log(ArLog::Normal, "A          -- awaits the completion of last issued positional command");
  ArLog::log(ArLog::Normal, "R          -- change pan/tilt movements to relative or absolute movements");
  ArLog::log(ArLog::Normal, "Z          -- move pan and tilt axes to zero");
  ArLog::log(ArLog::Normal, "M          -- Enter or Exit monitor mode");
  ArLog::log(ArLog::Normal, "H          -- Halt all motion");
  ArLog::log(ArLog::Normal, "S          -- print current variable values");
  ArLog::log(ArLog::Normal, "ESC        -- exit program");
  ArLog::log(ArLog::Normal, "\r\n");
}

void KeyPTU::status(void)
{
  ArLog::log(ArLog::Normal, "\r\nStatus:\r\n_________________\r\n");
  ArLog::log(ArLog::Normal, "Pan Position       = %.1f deg", myPTU.getPan());
  ArLog::log(ArLog::Normal, "Tilt Position      = %.1f deg", myPTU.getTilt());
  ArLog::log(ArLog::Normal, "Pan Slew Rate      = %d deg/sec", myPTU.getPanSlew());
  ArLog::log(ArLog::Normal, "Tilt Slew Rate     = %d deg/sec", myPTU.getTiltSlew());
  ArLog::log(ArLog::Normal, "Position Increment = %d deg", myPosIncrement);
  if (myAbsolute)
    ArLog::log(ArLog::Normal, "Positional-movements using absolute commands");
  else
    ArLog::log(ArLog::Normal, "Positional-movements using relative commands");
  ArLog::log(ArLog::Normal, "\r\n");
}

void KeyPTU::m(void)
{
  if (!myMonitor)
  {
    ArLog::log(ArLog::Normal, "Entering Monitor mode - hit 'M' to disable");
    myMonitor = true;
    myPTU.initMon(-60,60,30,-30);
  }
  else
  {
    myPTU.blank();	//Blank packet exits monitor mode
    myMonitor = false;
  }
}

void KeyPTU::h(void)
{
  myPTU.haltAll();
}

void KeyPTU::r(void)
{
  if (!myAbsolute)
  {
    myAbsolute = true;
  }
  else
  {
    myAbsolute = false;
  }
  status();
}


// the important function
void KeyPTU::drive(void)
{

  // if the PTU isn't initialized, initialize it here... it has to be 
  // done here instead of above because it needs to be done when the 
  // robot is connected
  if (!myPTUInited && myRobot->isConnected())
  {
    myPTU.init();
    myPTU.resetCalib();
    myPTU.awaitExec();
    myPTU.regStatPower();
    myPTU.regMotPower();
    mySlew = myPTU.getPanSlew(); //uses only pan slew rate
    myPTU.awaitExec();
    myPTUInited = true;
    myInit = false;
    myAbsolute = true;
  }

  if (myInit == true)
  {
    myPTU.init();
    myInit = false;
    myDesiredPanPos = myPTU.getPan();
    myDesiredTiltPos = myPTU.getTilt();
    mySlew = myPTU.getPanSlew(); //uses only pan slew rate
    myReset = false;
  }

  if (myReset == true)
  {
    myPTU.resetCalib();
    myPTU.awaitExec();
    myDesiredPanPos = myPTU.getPan();
    myDesiredTiltPos = myPTU.getTilt();
    myReset = false;
  }
  else
  {

    if (myDesiredPanPos != myPTU.getPan())
    {
      if (myAbsolute)
      	myPTU.pan(myDesiredPanPos);
      else
        myPTU.panRel(myDesiredPanPos - myPTU.getPan());
    }

    if (myDesiredTiltPos != myPTU.getTilt())
    {
      if (myAbsolute)
        myPTU.tilt(myDesiredTiltPos);
      else
        myPTU.tiltRel(myDesiredTiltPos - myPTU.getTilt());
    }

    if (mySlew != myPTU.getPanSlew())
    {
      myPTU.panSlew(mySlew);
      myPTU.tiltSlew(mySlew);
    }

  }

}

int main(int argc, char **argv) 
{ 
  Aria::init();

  // command-line arguments and robots connection
  ArArgumentParser argParser(&argc, argv);
  argParser.loadDefaultArguments();
  ArSimpleConnector con(&argParser);

  // the robot, but turn state reflection off (so we have no mobility control in
  // this program)
  ArRobot robot(NULL, false);

  // an object for keyboard control, class defined above, this also adds itself as a user task
  KeyPTU ptu(&robot);

  // parse command-line arguments (i.e. connection options for simple connector)
  if(!Aria::parseArgs())
  {
    Aria::logOptions();
    return 1;
  }

  // connect to the robot
  if (!con.connectRobot(&robot))
  {
    ArLog::log(ArLog::Terse, "Error connecting to robot!");
    Aria::shutdown();
    return 1;
  }


  // turn off the sonar
  robot.comInt(ArCommands::SONAR, 0);

  printf("Press '?' for available commands\r\n");

  // run, if we lose connection to the robot, exit
  robot.run(true);
  
  Aria::shutdown();
  return 0;
}

