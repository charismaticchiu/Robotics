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
#include <stdio.h>

/* Queries a subset of the ARIA classes for help text on their command line
 * arguments, outputs nd creates files containing that text, plus two files that
 * aggregates them all, one with some HTML formatting for use in the doxygen
 * documentation, one with an explanatory header for use as a stand alone
 * text file.  It redirects the log help output be reopening stdout bound to 
 * each file and then calling the logOptions() or similar methods on objects
 * of the classes with options.
 * 
 * When compiling this program, you must define either FOR_ARIA or
 * FOR_ARNETWORKING (or both) with -D, since this program can be used either 
 * for ARIA or ArNetworking.
 * 
 * The files generated are:
 *   docs/options/<class name>
 *   docs/options/all_options.dox
 *   CommandLineOptions.txt.in
 * 
 * If FOR_ARIA is defined, then the classes whose options are included are:
 *   ArRobotConnector
 *   ArLaserConnector
 *   ArGPSConnector
 *   ArCompassConnector
 *   [ArDaemonizer?]
 * 
 * If FOR_ARNETWORKING is defined, then these classes are also included:
 *   ArClientSimpleConnector
 *   ArServerSimpleOpener
 *   ArClientSwitchManager
 */



/* Wrapper classes provide a standardized publically accessible logOptions()
 * method for each class that have the ability to log options in some way. */

class Wrapper {
public:
  virtual void logOptions() = 0;
};


#ifdef FOR_ARIA
#include "ArArgumentParser.h"
#include "ArRobotConnector.h"
#include "ArGPSConnector.h"
#include "ArTCM2.h"

class ArRobotConnectorWrapper : 
  public ArRobotConnector,  public virtual Wrapper
{
public:
  ArRobotConnectorWrapper(ArArgumentParser *argParser) :
    ArRobotConnector(argParser, NULL)
  {
  }
  virtual void logOptions()
  {
    ArRobotConnector::logOptions();
  }
};


class ArLaserConnectorWrapper : 
  public ArLaserConnector,  public virtual Wrapper
{

  ArLMS2xx sickLaser;
  ArUrg urgLaser;
public:
  ArLaserConnectorWrapper(ArArgumentParser *argParser) :
    ArLaserConnector(argParser, NULL, NULL),
    sickLaser(1), urgLaser(1)
  {
  }
  virtual void logOptions()
  {
    puts(
"Laser types and options may also be set in the robot parameter file.  See the\n"
"ARIA reference documentation for details.\n\n"
"If a program supports multiple lasers, then options for additional lasers\n"
"after the first are given by appending the laser number (e.g. -laserType2)\n"
"To enable use of a laser, choose its type with the -laserType<N> options\n"
"(e.g.: -laserType lms2xx -laserType2 urg)\n\n" 
"The default laser type for the primary laser (laser 1) is \"lms2xx\".\n\n"
"Instruct a program to connect to a laser using the -connectLaser<N> option;\n"
"if a program requires use of a laser it usually always attempts to connect to\n"
"the primary laser, however.\n\n"
"The index number is optional in options for the primary laser (laser 1).\n\n"
);


    
    // sick
    puts("\nFor laser type \"lms2xx\":\n");
    addPlaceholderLaser(&sickLaser, 1);
    ArLaserConnector::logLaserOptions(myLasers[1], false, false);

    // urg
    puts("\nFor laser type \"urg\":\n");
    addPlaceholderLaser(&urgLaser, 1); // replace sick as first laser
    ArLaserConnector::logLaserOptions(myLasers[1], false, false);
  }
};

class ArGPSConnectorWrapper : 
  public ArGPSConnector, 
  public virtual Wrapper
{
public:
  ArGPSConnectorWrapper(ArArgumentParser *argParser) :
    ArGPSConnector(argParser)
  {
  }
  virtual void logOptions()
  {
    ArGPSConnector::logOptions();
  }
};

class ArCompassConnectorWrapper : 
  public ArCompassConnector, 
  public virtual Wrapper
{
public:
  ArCompassConnectorWrapper(ArArgumentParser *argParser) :
    ArCompassConnector(argParser)
  {
  }
  virtual void logOptions()
  {
    ArCompassConnector::logOptions();
  }
};
#endif

#ifdef FOR_ARNETWORKING
#include "ArNetworking.h"

class ArClientSimpleConnectorWrapper : public ArClientSimpleConnector, public virtual Wrapper
{
public:
  ArClientSimpleConnectorWrapper(ArArgumentParser *argParser) : ArClientSimpleConnector(argParser)
  {
  }
  virtual void logOptions()
  {
    ArClientSimpleConnector::logOptions();
  }
};

class ArServerSimpleOpenerWrapper : public ArServerSimpleOpener, public virtual Wrapper
{
public:
  ArServerSimpleOpenerWrapper(ArArgumentParser *argParser) : ArServerSimpleOpener(argParser)
  {
  }
  virtual void logOptions()
  {
    ArServerSimpleOpener::logOptions();
  }
};

class ArClientSwitchManagerWrapper : public ArClientSwitchManager, public virtual Wrapper
{
public:
  ArClientSwitchManagerWrapper(ArServerBase *server, ArArgumentParser *argParser) : ArClientSwitchManager(server, argParser)
  {
  }
  virtual void logOptions()
  {
    ArClientSwitchManager::logOptions();
  }
};
#endif

const char *EXPLANATION = 
"Some classes in ARIA and ArNetworking check a program's run time options to\n"
"specify parameters and options. These options are used to configure run time\n"
"accessory device parameters (ports, speeds, etc.) used by ARIA; host names,\n"
"port numbers, etc. used by ArNetworking; and various other run time options.\n"
"Options may be given as program arguments on the command line, or globally\n"
"saved as defaults in the file /etc/Aria.args if on Linux, or in the ARIAARGS\n"
"environment variable.  Arguments given on the command line may override some\n"
"internal defaults or values read from the robot parameter files.\n\n"
"Note, an option will be available only in programs that instantiate an\n"
"object of the class that uses it. Some programs may also check for\n"
"program-specific command line options.\n\n"
"Use the special \"-help\" command line option to cause a program to \n"
"print out its available options.\n\n"
"A list of options used by each class follows.\n\n";

/* Redirect stdout to a file. If reopening stdout for the new file fails, print
 * a message and exit the program with error code 3.
 */
void redirectStdout(const char *filename)
{
  FILE *fp = freopen(filename, "w", stdout);
  if(fp == NULL)
  {
    fprintf(stderr, "genCommandLineOptionDocs: Error opening \"%s\"!  Exiting.\n", filename);
    Aria::exit(3);
  }
}

typedef std::pair<std::string, Wrapper*> WrapPair;
typedef std::vector<WrapPair> WrapList;

int main(int argc, char **argv)
{
  Aria::init();
  ArArgumentParser argParser(&argc, argv);

  WrapList wrappers;

#ifdef FOR_ARIA
  wrappers.push_back(WrapPair("ArRobotConnector", new ArRobotConnectorWrapper(&argParser)));
  wrappers.push_back(WrapPair("ArLaserConnector", new ArLaserConnectorWrapper(&argParser)));
  wrappers.push_back(WrapPair("ArGPSConnector", new ArGPSConnectorWrapper(&argParser)));
  wrappers.push_back(WrapPair("ArCompassConnector", new ArCompassConnectorWrapper(&argParser)));
#endif

#ifdef FOR_ARNETWORKING
  ArServerBase server;
  wrappers.push_back(WrapPair("ArClientSimpleConnector", new ArClientSimpleConnectorWrapper(&argParser)));
  wrappers.push_back(WrapPair("ArServerSimpleOpener", new ArServerSimpleOpenerWrapper(&argParser)));
  wrappers.push_back(WrapPair("ArClientSwitchManager", new ArClientSwitchManagerWrapper(&server, &argParser)));
#endif

  /* Write docs/options/all_options.dox */
  // TODO process text to replace HTML characters with entities or formatting
  // (especially < and >)
  redirectStdout("docs/options/all_options.dox");
  printf("/* This file was automatically generated by utils/genCommandLineOptionDocs.cpp. Do not modify or changes will be lost.*/\n\n"\
    "/** @page CommandLineOptions Command Line Option Summary\n\n%s\n\n", EXPLANATION);
  // TODO process text by turning it into a <dl> or similar: start new <dt> at
  // beginning of line, add </dt><dd> at first \t, then add </dt> at end.
  for(WrapList::const_iterator i = wrappers.begin(); i != wrappers.end(); ++i)
  {
    printf("@section %s\n\n(See %s for class documentation)\n\n<pre>", (*i).first.c_str(), (*i).first.c_str());
    (*i).second->logOptions();
    puts("</pre>\n");
    fprintf(stderr, "genCommandLineOptionDocs: Added %s to docs/options/all_options.dox\n", (*i).first.c_str());
  }
  puts("*/");
  fputs("genCommandLineOptionDocs: Wrote docs/options/all_options.dox\n", stderr);


  /* Write docs/options/<class> */
  for(WrapList::const_iterator i = wrappers.begin(); i != wrappers.end(); ++i)
  {
    std::string filename("docs/options/");
    filename += (*i).first + "_options";
    redirectStdout(filename.c_str());
    (*i).second->logOptions();
    fprintf(stderr, "genCommandLineOptionDocs: Wrote %s\n", filename.c_str());
  }

  /* Write CommandLineOptions.txt.in */
  redirectStdout("CommandLineOptions.txt.in");
#ifdef FOR_ARIA
  puts("\nARIA $ARIAVERSION\n");
#elif defined(FOR_ARNETWORKING)
  puts("\nArNetworking $ARIAVERSION\n");
#endif
  printf("Summary of command line options\n\n%s", EXPLANATION);

  for(WrapList::const_iterator i = wrappers.begin(); i != wrappers.end(); ++i)
  {
    puts("");
    puts((*i).first.c_str());
    for(std::string::size_type c = 0; c < (*i).first.size(); ++c)
      fputc('-', stdout);
    puts("");
    (*i).second->logOptions();
  }
  puts("\n");
  fputs("genCommandLineOptionDocs: Wrote CommandLineOptions.txt.in\n", stderr);

  Aria::exit(0);
}
