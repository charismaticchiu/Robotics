####
#### Aria top level makefile
####
####
#### Internal variables:
####
#### CFILEEXT -- This is to denote the file extension that is used. This is
####             used by the build rules to figure out which files to build.
####               ie: cpp
####               ie: cxx
####               ie: cc

# Make sure we're using a compiler (we probably don't need to define
# it but we have been so here it is unless it was already set)
ifndef CXX
CXX:=g++
endif

####
#### General variables
####

CFILEEXT:=cpp
# this is set up with the extra layer since the python wrapper needs exceptions
# but I didn't want to have two sets of defines
BARECXXFLAGS:= -fPIC -g -Wall -D_REENTRANT 
CXXFLAGS+=$(BARECXXFLAGS) -fno-exceptions 

CXXINC:=-Iinclude
CXXLINK:=-Llib -lAria -lpthread -ldl -lrt

CXXSTATICLINK:=-Llib -Xlinker -Bstatic -lAria -Xlinker -Bdynamic -lpthread -ldl -lrt -Xlinker -Bstatic -lstdc++ -Xlinker -Bdynamic


ifdef JAVA_BIN
JAVAC:=$(JAVA_BIN)/javac
JAR:=$(JAVA_BIN)/jar
else
JAVAC:=javac
JAR:=jar
endif

####
#### Aria build variables
####

# Default targets to build in the default rule:
TARGETS:=lib/libAria.so examples/demo

# Lots of targets, to build in the everything rule:
ALL_TARGETS:=lib/libAria.so utils examples tests docs params lib/libArNetworking.so swig arnetworking_docs arnetworking_swig lib/libAria.a lib/libArNetworking.a examples/demoStatic

CFILES:= \
	ArAction.cpp \
	ArActionAvoidFront.cpp \
	ArActionAvoidSide.cpp \
	ArActionBumpers.cpp \
	ArActionColorFollow.cpp \
	ArActionConstantVelocity.cpp \
	ArActionDeceleratingLimiter.cpp \
	ArActionDesired.cpp \
	ArActionDriveDistance.cpp \
	ArActionGoto.cpp \
	ArActionGotoStraight.cpp \
	ArActionGroup.cpp \
	ArActionGroups.cpp \
	ArActionInput.cpp \
	ArActionIRs.cpp \
	ArActionJoydrive.cpp \
	ArActionKeydrive.cpp \
	ArActionLimiterBackwards.cpp \
	ArActionLimiterForwards.cpp \
	ArActionLimiterTableSensor.cpp \
	ArActionMovementParameters.cpp \
	ArActionRatioInput.cpp \
	ArActionRobotJoydrive.cpp \
	ArActionStallRecover.cpp \
	ArActionStop.cpp \
	ArActionTriangleDriveTo.cpp \
	ArActionTurn.cpp \
	ArACTS.cpp \
	ArAMPTU.cpp \
	ArAnalogGyro.cpp \
	ArArg.cpp \
	ArArgumentBuilder.cpp \
	ArArgumentParser.cpp \
	ArASyncTask.cpp \
	ArBasePacket.cpp \
	ArBumpers.cpp \
	ArCameraCommands.cpp \
	ArCameraCollection.cpp \
	ArCondition_LIN.cpp \
	ArConfig.cpp \
	ArConfigArg.cpp \
	ArConfigGroup.cpp \
	ArDataLogger.cpp \
	ArDeviceConnection.cpp \
	ArDPPTU.cpp \
	ArFileParser.cpp \
	ArForbiddenRangeDevice.cpp \
	ArFunctorASyncTask.cpp \
	ArGPS.cpp \
	ArGPSConnector.cpp \
	ArGPSCoords.cpp \
	ArGripper.cpp \
	ArInterpolation.cpp \
	ArIrrfDevice.cpp \
	ArIRs.cpp \
	ArJoyHandler.cpp \
	ArJoyHandler_LIN.cpp \
	ArKeyHandler.cpp \
	ArLaser.cpp \
	ArLaserConnector.cpp \
	ArLaserFilter.cpp \
	ArLaserLogger.cpp \
	ArLaserReflectorDevice.cpp \
	ArLineFinder.cpp \
	ArLMS1XX.cpp \
	ArLMS2xx.cpp \
	ArLMS2xxPacket.cpp \
	ArLMS2xxPacketReceiver.cpp \
	ArLog.cpp \
	ArLogFileConnection.cpp \
	ArMap.cpp \
	ArMapComponents.cpp \
	ArMapInterface.cpp \
	ArMapObject.cpp \
	ArMapUtils.cpp \
	ArMD5Calculator.cpp \
	ArMode.cpp \
	ArModes.cpp \
	ArModule.cpp \
	ArModuleLoader.cpp \
	ArMutex.cpp \
	ArMutex_LIN.cpp \
	ArNetServer.cpp \
	ArNMEAParser.cpp \
	ArNovatelGPS.cpp \
	ArP2Arm.cpp \
	ArPriorityResolver.cpp \
	ArPTZ.cpp \
	ArRangeBuffer.cpp \
	ArRangeDevice.cpp \
	ArRangeDeviceThreaded.cpp \
	ArRatioInputKeydrive.cpp \
	ArRatioInputJoydrive.cpp \
	ArRatioInputRobotJoydrive.cpp \
	ArRecurrentTask.cpp \
	ArRobot.cpp \
	ArRobotConfig.cpp \
	ArRobotConfigPacketReader.cpp \
	ArRobotConnector.cpp \
	ArRobotJoyHandler.cpp \
	ArRobotPacket.cpp \
	ArRobotPacketReceiver.cpp \
	ArRobotPacketSender.cpp \
	ArRobotParams.cpp \
	ArRobotTypes.cpp \
	ArRVisionPTZ.cpp \
	ArSerialConnection_LIN.cpp \
	ArSick.cpp \
	ArSignalHandler_LIN.cpp \
	ArSimpleConnector.cpp \
	ArSimulatedLaser.cpp \
	ArSocket.cpp \
	ArSocket_LIN.cpp \
	ArSonarDevice.cpp \
	ArSensorReading.cpp \
	ArSonyPTZ.cpp \
	ArSoundsQueue.cpp \
	ArSoundPlayer.cpp \
	ArStringInfoGroup.cpp \
	ArSyncLoop.cpp \
	ArSyncTask.cpp \
	ArSystemStatus.cpp \
	ArSonarAutoDisabler.cpp \
	ArSpeech.cpp \
	ArTCM2.cpp \
	ArTCMCompassDirect.cpp \
	ArTCMCompassRobot.cpp \
	ArTcpConnection.cpp \
	ArThread.cpp \
	ArThread_LIN.cpp \
	ArTransform.cpp \
	ArTrimbleGPS.cpp \
	ArUrg.cpp \
	ArVersalogicIO.cpp \
	ArVCC4.cpp \
	Aria.cpp \
	ariaUtil.cpp \
	md5.cpp


####
#### Utility variables. No need to touch these.
####

OTFILES:=$(patsubst %.$(CFILEEXT),%.o,$(CFILES))
OFILES:=$(patsubst %,obj/%,$(OTFILES))
EXAMPLES_CPP:=$(shell find examples -name "*.$(CFILEEXT)" | grep -v Mod.cpp | grep -v proprietary)
EXAMPLES:=$(patsubst %.$(CFILEEXT),%,$(EXAMPLES_CPP))
MOD_EXAMPLES_CPP:=$(shell find examples -name "*.$(CFILEEXT)" | grep Mod.cpp)
MOD_EXAMPLES:=$(patsubst %.$(CFILEEXT),%.so,$(MOD_EXAMPLES_CPP))
TESTS_CPP:=$(shell find tests -name "*.$(CFILEEXT)" | grep -v Mod.cpp | grep -v proprietary)
TESTS:=$(patsubst %.$(CFILEEXT),%,$(TESTS_CPP))
ADVANCED_CPP:=$(shell find advanced -name "*.$(CFILEEXT)" | grep -v Mod.cpp | grep -v proprietary)
ADVANCED:=$(patsubst %.$(CFILEEXT),%,$(ADVANCED_CPP))
UTILS_CPP:=$(shell find utils -name "*.$(CFILEEXT)")
UTILS:=$(patsubst %.$(CFILEEXT),%,$(UTILS_CPP))
SRC_FILES:=$(patsubst %,src/%,$(CFILES))
HEADER_FILES:=$(shell find include -type f -name \*.h)

####
#### General rules for user invocation
####

# Default Rule
all: dirs $(TARGETS)

# Build all targets, docs, params, etc. etc.
everything: dirs $(ALL_TARGETS) 


examples: $(EXAMPLES) 

modExamples: $(MOD_EXAMPLES)

tests: $(TESTS)

advanced: $(ADVANCED)

utils: $(UTILS)

cleanDep:
	-rm Makefile.dep `find . -name Makefile.dep`

# directories that might not exist:
dirs:
	@mkdir -p -v obj
	@mkdir -p -v lib

docs: doc
doc: docs/index.html
docs/index.html: $(SRC_FILES) $(HEADER_FILES) $(EXAMPLES_CPP) doxygen.conf docs/overview.dox docs/options/all_options.dox docs/params.dox
	$(MAKE) cleanDoc
	doxygen doxygen.conf

arnetworking_swig:
	$(MAKE) -C ArNetworking java python

arnetworking_docs:
	$(MAKE) -C ArNetworking docs

help:
	@echo To make most things, run \'make\' or \'make all\'
	@echo Some useful targets include: 
	@echo "  clean"
	@echo "  dep"
	@echo "  cleanDep"
	@echo "  docs"
	@echo "  cleanAll (also cleans ArNetworking, java, python, etc.)"
	@echo "  examples"
	@echo "  tests"
	@echo "  utils"
	@echo "  python" 
	@echo "  cleanPython"
	@echo "  java" 
	@echo "  cleanJava"
	@echo "  examples/<Some Example>, where examples/<Some Example>.cpp is an example program source file."
	@echo "  tests/<Some Test>, where tests/<Some Test>.cpp is a test program source file."
	@echo "  allLibs (try to build all auxilliary libraries that you have installed by running make in each directory starting with \"Ar\")"
	@echo "  cleanAllLibs, depAllLibs (do make clean or make dep in the \"Ar*\" auxilliary libraries)"
	@echo "  install (if this is a source tar.gz package)"

info:
	@echo ARIA=$(ARIA)
	@echo CXX=$(CXX)
	@echo CXXFLAGS=$(CXXFLAGS)
	@echo CXXINC=$(CXXINC)
	@echo CXXLINK=$(CXXLINK)
	@echo CXXSTATICLINK=$(CXXSTATICLINK)
	@echo ALL_TARGETS=$(ALL_TARGETS)
	@echo
	@echo JAVAC=$(JAVAC)
	@echo JAR=$(JAR)
	@echo
	@echo Use \'make moreinfo\' for info about individual files, etc.

moreinfo:
	@echo CFILES=$(CFILES)
	@echo
	@echo EXAMPLES=$(EXAMPLES)
	@echo
	@echo TESTS=$(TESTS)
	@echo
	@echo ADVANCED=$(ADVANCED)
	@echo
	@echo UTILS=$(UTILS)
	@echo
	@echo SRC_FILES=$(SRC_FILES)
	@echo
	@echo HEADER_FILES=$(HEADER_FILES)



clean: cleanUtils cleanExamples cleanModules cleanTests cleanAdvanced 
	-rm -f lib/libAria.a lib/libAria.so $(OFILES) `find . -name core` `find . -name '*~'` obj/AriaPy.o obj/AriaJava.o

cleanUtils:
	-rm -f $(UTILS)

cleanExamples:
	-rm -f $(EXAMPLES)

cleanTests:
	-rm -f $(TESTS)

cleanAdvanced:
	-rm -f $(ADVANCED)

cleanModules:
	-rm -f $(MOD_EXAMPLES)

cleanDoc:
	-rm -rf docs/*.html docs/*.png docs/doxygen.css

cleanPython:
	-rm python/_AriaPy.so
	-rm python/AriaPy.py
	-rm python/AriaPy.pyc
	-rm python/AriaPy_wrap.cpp
	-rm python/AriaPy_wrap.h
	-rm obj/AriaPy_wrap.o

dep: clean 
	if [ -f `echo src/*.cpp | cut -d' ' -f1` ]; then \
	$(CXX) $(CXXFLAGS) $(CXXINC) -MM src/*.cpp | \
	awk '$$1 ~ /:/{printf "obj/%s\n", $$0} $$1 !~ /:/' > Makefile.dep; fi
	if [ -f `echo examples/*.cpp | cut -d' ' -f1` ]; then \
	$(CXX) $(CXXFLAGS) $(CXXINC) -MM examples/*.cpp | \
	awk '$$1 ~ /:/{printf "examples/%s\n", $$0} $$1 !~ /:/' | \
	sed 's/\.o//' >> Makefile.dep; fi
	if [ -f `echo utils/*.cpp | cut -d' ' -f1` ]; then \
	$(CXX) $(CXXFLAGS) $(CXXINC) -MM utils/*.cpp | \
	awk '$$1 ~ /:/{printf "utils/%s\n", $$0} $$1 !~ /:/' | \
	sed 's/\.o//' >> Makefile.dep; fi
	if [ -f `echo tests/*.cpp | cut -d' ' -f1` ]; then \
	$(CXX) $(CXXFLAGS) $(CXXINC) -MM tests/*.cpp | \
	awk '$$1 ~ /:/{printf "tests/%s\n", $$0} $$1 !~ /:/' | \
	sed 's/\.o//' >> Makefile.dep; fi
	if [ -f `echo advanced/*.cpp | cut -d' ' -f1` ]; then \
	$(CXX) $(CXXFLAGS) $(CXXINC) -MM advanced/*.cpp | \
	awk '$$1 ~ /:/{printf "advanced/%s\n", $$0} $$1 !~ /:/' | \
	sed 's/\.o//' >> Makefile.dep; fi

Makefile.dep:
	$(MAKE) dep

depAllLibs:
	make dep;
	for dir in `find . -maxdepth 1 -name "Ar*" -xtype d`; do cd $$dir; make dep; cd ..; done

cleanAll: clean cleanJava cleanPython cleanDep
	$(MAKE) -C ArNetworking cleanAll



params: utils/makeParams
	-mkdir params
	utils/makeParams

CommandLineOptions.txt.in docs/options/all_options.dox: utils/genCommandLineOptionDocs src/ArSimpleConnector.cpp src/ArGPSConnector.cpp src/ArTCM2.cpp
	-mkdir docs/options
	utils/genCommandLineOptionDocs

alllibs: allLibs

allLibs: all 
	find . -type d -and -name Ar\* -maxdepth 1 -exec $(MAKE) -C \{\}  \;

cleanAllLibs: clean cleanJava cleanPython cleanDep
	for dir in `find . -maxdepth 1 -name "Ar*" -xtype d`; do $(MAKE) -C $$dir cleanAll; done

cleanalllibs: cleanAllLibs

checkAll:
	$(MAKE) everything > checkAll.log 2>&1
	$(MAKE) allLibs >> checkAll.log 2>&1
	grep -n -v 'Error 1 (ignored)' checkAll.log | grep -C 4 'Error' > checkAll-Errors.log
	@echo
	@echo Errors were:
	@echo
	@cat checkAll-Errors.log
	@echo
	@echo Error log also saved as checkAll-Errors.log.  Full log saved as checkAll.log.

####
#### Swig wrappers
####


### Python wrapper: ###

python: python/_AriaPy.so python/AriaPy.py

py: python

python-doc: python/AriaPy.html

python/AriaPy.html: python/AriaPy.py
	cd python; pydoc -w AriaPy

python/AriaPy_wrap.cpp python/AriaPy.py: include/wrapper.i include/*.h
	-rm -f `find python -maxdepth 1 -xtype f -name "*Aria*" | grep -v .ds | grep -v .sln | grep -v .vcproj`
	cd python; swig -Wall -c++ -python -classic -module AriaPy -Dlinux -DAREXPORT -o AriaPy_wrap.cpp -I../include ../include/wrapper.i 

# This is the old way that would compile in all the object files, it is 
# deprecated in favor of the way below

#python/_AriaPy.so: obj/AriaPy_wrap.o $(OFILES) Makefile.dep
#	@echo  == Building loadable module for python with `$(CXX) --version`
#	$(CXX) -shared -o $(@) $(OFILES) obj/AriaPy_wrap.o -lpthread -ldl -lrt

# This is the new way that just links in the aria lib instead (so this
# will work for other libraries too)
python/_AriaPy.so: obj/AriaPy_wrap.o lib/libAria.so Makefile.dep
	$(CXX) -shared -o $(@) obj/AriaPy_wrap.o -lpthread -ldl -lrt -Llib -lAria

ifdef PYTHON_INCLUDE
PYTHON_INCLUDE_FLAGS=-I$(PYTHON_INCLUDE)
endif

obj/AriaPy_wrap.o: python/AriaPy_wrap.cpp
	mkdir -p obj
	@ if test -z "$(PYTHON_INCLUDE)"; then echo "*** Warning: PYTHON_INCLUDE is not set, building AriaPy_wrap.cpp may fail"; fi
	$(CXX) -c $(BARECXXFLAGS) $(CXXINC) $(PYTHON_INCLUDE_FLAGS) $< -o $@


### Java Wrapper: ###



java:  lib/libAriaJava.so java/Aria.jar

java/Aria.jar: java/com/mobilerobots/Aria/ArRobot.class
	cd java; $(JAR) cf Aria.jar com/mobilerobots/Aria/*.class

cleanJava:
	-rm -f `find java -xtype f -maxdepth 1 | grep -v .ds | grep -v .sln | grep -v .vcproj` 
	-rm lib/libAriaJava.so
	-rm java/AriaJava_wrap.cpp
	-rm java/AriaJava_wrap.h
	-rm obj/AriaJava_wrap.o

cleanSwigJava:
	-rm -r java/Aria.jar java/com/mobilerobots/Aria

lib/libAriaJava.so: obj/AriaJava_wrap.o lib/libAria.so Makefile.dep
	$(CXX) -shared -o $(@) obj/AriaJava_wrap.o -lpthread -ldl -lrt -Llib -lAria

obj/AriaJava_wrap.o: java/AriaJava_wrap.cpp
	@ if test -z "$(JAVA_INCLUDE)"; then echo "Error: JAVA_INCLUDE is not set, compiling AriaJava_wrap.cpp will fail!"; fi
	@ test -n "$(JAVA_INCLUDE)"
	@ if test \! -d "$(JAVA_INCLUDE)"; then echo "Error: JAVA_INCLUDE directory $(JAVA_INCLUDE) does not exist, compiling AriaJava_wrap.cpp will fail!"; fi
	@ test -d "$(JAVA_INCLUDE)" 
	mkdir -p obj
	$(CXX) -c $(BARECXXFLAGS) $(CXXINC) -I$(JAVA_INCLUDE) -I$(JAVA_INCLUDE)/linux $< -o $@

java/AriaJava_wrap.cpp java/com/mobilerobots/Aria/ArRobot.java: include/wrapper.i $(HEADER_FILES)
	-mkdir -p java/com/mobilerobots/Aria
	rm java/com/mobilerobots/Aria/*.java java/AriaJava_wrap.cpp java/AriaJava_wrap.h;  swig -Wall -c++ -java -package com.mobilerobots.Aria -outdir java/com/mobilerobots/Aria -module AriaJava -Dlinux -DAREXPORT -o java/AriaJava_wrap.cpp -Iinclude include/wrapper.i 
    
# The sed script is a hack to let subclasses outside of Aria work for ArFunctor;
# for some reason SWIG doesn't use my %typmeap(javabody) for ArFunctor. 
#    \
#	  && sed 's/protected static long getCPtr/public ArFunctor() { this(0, false); }     public static long getCPtr/' java/com/mobilerobots/Aria/ArFunctor.java > ArFunctor.java.tmp  \
#		&& mv ArFunctor.java.tmp java/com/mobilerobots/Aria/ArFunctor.java

# It is much faster to compile them all at once. Use ArRobot.java/class as a
# stand-in for all java files and all class files.
java/com/mobilerobots/Aria/ArRobot.class: java/com/mobilerobots/Aria/ArRobot.java
		rm java/com/mobilerobots/Aria/*.class; $(JAVAC) -classpath java java/com/mobilerobots/Aria/*.java 


java/ArNetworking.jar: FORCE
	$(MAKE) -C ArNetworking ../$@

lib/libArNetworkingJava.so: FORCE
	$(MAKE) -C ArNetworking ../$@

python/ArNetworkingPy.py: FORCE
	$(MAKE) -C ArNetworking ../$@

python/_ArNetworkingPy.so: FORCE
	$(MAKE) -C ArNetworking ../$@


####
#### Targets to actually build binaries (libraries, programs)
####

lib/libAria.so: $(OFILES) Makefile.dep 
	$(CXX) -shared -o $(@) $(OFILES)

lib/libAria.a: $(OFILES) Makefile.dep 
	ar -cr $(@) $(OFILES)
	ranlib $(@)

examples/%.so: examples/%.$(CFILEEXT) lib/libAria.so Makefile.dep
	$(CXX) $(CXXFLAGS) $(CXXINC) -shared $< -o $@

examples/%: examples/%.$(CFILEEXT) lib/libAria.so Makefile.dep
	$(CXX) $(CXXFLAGS) $(CXXINC) $< -o $@ $(CXXLINK)

examples/%Static: examples/%.$(CFILEEXT) lib/libAria.a Makefile.dep
	$(CXX) $(CXXFLAGS) $(CXXINC) $< -o $@ $(CXXSTATICLINK)
	strip $@

tests/%.so: tests/%.$(CFILEEXT) lib/libAria.so Makefile.dep
	$(CXX) $(CXXFLAGS) $(CXXINC) -shared $< -o $@

tests/%: tests/%.$(CFILEEXT) lib/libAria.so Makefile.dep
	$(CXX) $(CXXFLAGS) $(CXXINC) $< -o $@ $(CXXLINK)

tests/%Static: tests/%.$(CFILEEXT) lib/libAria.a Makefile.dep
	$(CXX) $(CXXFLAGS) $(CXXINC) $< -o $@ $(CXXSTATICLINK)
	strip $@

advanced/%.so: advanced/%.$(CFILEEXT) lib/libAria.so
	$(CXX) $(CXXFLAGS) $(CXXINC) -shared $< -o $@

advanced/%: advanced/%.$(CFILEEXT) lib/libAria.so Makefile.dep
	$(CXX) $(CXXFLAGS) $(CXXINC) $< -o $@ $(CXXLINK)

advanced/%Static: advanced/%.$(CFILEEXT) lib/libAria.a Makefile.dep
	$(CXX) $(CXXFLAGS) $(CXXINC) $< -o $@ $(CXXSTATICLINK)
	strip $@

utils/%: utils/%.$(CFILEEXT) lib/libAria.so Makefile.dep
	$(CXX) $(CXXFLAGS) $(CXXINC) $< -o $@ $(CXXLINK)

utils/%Static: utils/%.$(CFILEEXT) lib/libAria.a Makefile.dep
	$(CXX) $(CXXFLAGS) $(CXXINC) $< -o $@ $(CXXSTATICLINK)
	strip $@

utils/genCommandLineOptionDocs: utils/genCommandLineOptionDocs.cpp lib/libAria.so Makefile.dep
	$(CXX) $(CXXFLAGS) -DFOR_ARIA $(CXXINC) $< -o $@ $(CXXLINK)

obj/%.o : src/%.cpp Makefile.dep
	@mkdir -p obj
	$(CXX) -c $(CXXFLAGS) $(CXXINC) $< -o $@

obj/%.o : src/%.c Makefile.dep
	@mjdir -p obj
	$(CXX) -c $(CXXFLAGS) $(CXXINC) $< -o $@


# Don't build .o files if their library is up to date with respect to source files:
.INTERMEDIATE: $(OFILES)

# But don't delete .o files if we do make them in order to make a library:
.PRECIOUS: $(OFILES)


# To build things in the ArNetworking subdirectory:

lib/libArNetworking.so: FORCE
	$(MAKE) -C ArNetworking ../$@

lib/libArNetworking.a: FORCE 
	$(MAKE) -C ArNetworking ../$@

ArNetworking/examples/%: ArNetworking/examples/%.cpp lib/libArNetworking.so
	$(MAKE) -C ArNetworking examples/$*

ArNetworking/examples/%Static: ArNetworking/examples/%.cpp lib/libArNetworking.a
	$(MAKE) -C ArNetworking examples/$*Static

ArNetworking/docs/index.html: ArNetworking/doxygen.conf FORCE
	$(MAKE) -C ArNetworking docs/index.html

FORCE:


####
#### Installation and distribution 
####


ifndef INSTALL_DIR
INSTALL_DIR=/usr/local/Aria
endif

ifndef SYSTEM_ETC_DIR
SYSTEM_ETC_DIR=/etc
endif

# What to put in /etc/Aria:
ifndef STORED_INSTALL_DIR
STORED_INSTALL_DIR=$(INSTALL_DIR)
endif

# How to run 'install' for the install rule:
ifndef INSTALL
INSTALL:=install --preserve-timestamps
endif

# Install rule.  This can be used by users or ARIA developers; in the latter
# case it also installs various files needed to make a release distribution.
# Override installation locations with INSTALL_DIR environment variable.
# Things are installed group-writable so as to be hacked upon.
install:
	@echo      	--------------------------------------
	@echo		    Installing ARIA in $(DESTDIR)$(INSTALL_DIR)...
	@echo      	--------------------------------------
	$(INSTALL) -m 775 -d $(DESTDIR)$(INSTALL_DIR)
	find    include src tests utils params docs examples advanced maps \
	        ArNetworking java javaExamples python pythonExamples  obj \
	        \( -name \*.o -or -name core -or -name CVS -or -name .\* -or -name \*~ -or -name tmp -or -name proprietary* -or -name \*.bak -or -name \*.class \) -prune  \
	        -or -type d   -exec $(INSTALL) -d -m 777 $(DESTDIR)$(INSTALL_DIR)/\{\} \; \
	        -or -type l   -exec cp --no-dereference \{\} $(DESTDIR)$(INSTALL_DIR)/\{\} \; \
	        -or -name \*.a -exec $(INSTALL) -D -m 666 \{\}  $(DESTDIR)$(INSTALL_DIR)/\{\} \; \
	        -or -perm +1  -exec $(INSTALL) -D --strip -m 777 \{\}  $(DESTDIR)$(INSTALL_DIR)/\{\} \; \
	        -or           -exec $(INSTALL) -D -m 666 \{\} $(DESTDIR)$(INSTALL_DIR)/\{\} \;
	$(INSTALL) -D -m 664 LICENSE.txt INSTALL.txt README.txt Makefile Aria-Reference.html version.txt Changes.txt CommandLineOptions.txt $(DESTDIR)$(INSTALL_DIR)/
	$(INSTALL) -D -m 666  Makefile.dep doxygen.conf $(DESTDIR)$(INSTALL_DIR)/
	$(INSTALL) -D -m 664 bin/SIMULATOR_README.txt $(DESTDIR)$(INSTALL_DIR)/bin/SIMULATOR_README.txt
	$(INSTALL) -d -m 777 $(DESTDIR)$(INSTALL_DIR)/lib/
	$(INSTALL) -D --strip -m 666 lib/libAria.so lib/libArNetworking.so lib/libAriaJava.so lib/libArNetworkingJava.so $(DESTDIR)$(INSTALL_DIR)/lib/
	@if test -z "$(DIST_INSTALL)"; then \
		echo "if test \! -d $(DESTDIR)$(SYSTEM_ETC_DIR); then install -d $(DESTDIR)$(SYSTEM_ETC_DIR); fi" ;\
		if test \! -d $(DESTDIR)$(SYSTEM_ETC_DIR); then install -d $(DESTDIR)$(SYSTEM_ETC_DIR); fi ;\
		echo "echo $(STORED_INSTALL_DIR) > $(DESTDIR)$(SYSTEM_ETC_DIR)/Aria" ;\
		echo $(STORED_INSTALL_DIR) > $(DESTDIR)$(SYSTEM_ETC_DIR)/Aria ;\
		echo       ------------------------------------------------------------------------------------ ;\
		echo       ARIA has been installed in $(DESTDIR)$(INSTALL_DIR). ;\
		echo ;\
		echo       To be able to use the ARIA libraries, you must now add $(DESTDIR)$(INSTALL_DIR)/lib ;\
		echo       to your LD_LIBRARY_PATH environment variable, or to the /etc/ld.so.conf system file, ;\
		echo       then run \'ldconfig\';\
		echo     	 ------------------------------------------------------------------------------------ ;\
	fi

#		echo ;\
#		echo       The files are owned by the group \"users\", and all members of that group ;\
#		echo       can enter the directory, read files, and modify the \"examples\" directory. ;\

###
### Make optimization, tell it what rules aren't files:
###
.PHONY: all everything examples modExamples tests advanced utils cleanDep docs doc dirs help info moreinfo clean cleanUtils cleanExamples cleanTests cleanAdvanced cleanModules cleanDoc cleanPython dep depAll cleanAll params allLibs python python-doc java cleanJava install alllibs arnetworking_swig arnetworking_docs params swig help info moreinfo py python-doc cleanSwigJava dist-install checkAll

### Autogenerated dependencies:
# Just see if there is a Makefile.dep, if so include one... there
# should be one because various rules above depend on Makefile.dep.
# Note, use this ifeq/wildcard hacok rather than "-include" directive to 
# prevent Make from trying to create Makefile.dep itself if missing during the
# recursive call to $(MAKE) dep.  (it just keeps
# calling make dep recursively).
ifeq (Makefile.dep,$(wildcard Makefile.dep))
include Makefile.dep
# End of the Makefile.dep check
endif


### dist
# Include rules for creating distribution packages, if enabled
# (for MobileRobots internal use)
ifdef ARIA_DIST
ifeq (dist/Makefile.aria-dist,$(wildcard dist/Makefile.aria-dist))
$(info ARIA_DIST variable is set and dist/Makefile.aria-dist exists, including it)
include dist/Makefile.aria-dist
else
$(warning ARIA_DIST variable is set but dist/Makefile.aria-dist not found, skipping it)
endif
else
rpm deb tgz windist dev-rpm dev-deb dev-tgz dev-windist rpm-all deb-all windist-all tgz-all dev-rpm-all dev-deb-all dev-tgz-all dev-windist-all testinst distinfo:
	$(error Define ARIA_DIST variable to enable package building rules)
endif
