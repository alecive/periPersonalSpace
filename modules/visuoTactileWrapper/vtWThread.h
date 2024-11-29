/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 * This thread detects a touched taxel on the skin (through readings from the
 * skinContactList port), and it moves the "controlateral" limb toward
 * the affected taxel.
*/

#ifndef __VTWTHREAD_H__
#define __VTWTHREAD_H__

#include <yarp/os/Time.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Log.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/all.h>

#include <gsl/gsl_math.h>

#include <iCub/iKin/iKinFwd.h>

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdarg.h>
#include <vector>

//#include <cv.h>
//#include <highgui.h>
#include <iCub/ctrl/math.h>
#include <iCub/periPersonalSpace/utils.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::iKin;
using namespace iCub::ctrl;

using namespace std;

class vtWThread: public PeriodicThread
{
protected:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the module (to change port names accordingly):
    string name;
    // Name of the robot (to address the module toward icub or icubSim):
    string robot;
    // Resource finder used to find for files and configurations
    ResourceFinder* rf;

    /***************************************************************************/
    // INTERNAL VARIABLES:
    int counter{0};
    // Output Port & events
        yarp::os::Port depth2kinPort;
        yarp::os::Port SFMPort;
        BufferedPort<Bottle>   eventsPort;
        vector <IncomingEvent> events;

    // Input Ports & related data
        BufferedPort<Bottle>  pf3dTrackerPort;
        Bottle               *pf3dTrackerBottle;
        Vector                pf3dTrackerPos;
        Vector                pf3dTrackerVel;

        BufferedPort<Bottle>  optFlowPort;
        Bottle               *optFlowBottle;
        Vector                optFlowPos;
        Vector                optFlowVel;

        BufferedPort<Bottle>  doubleTouchPort;           
        Bottle               *doubleTouchBottle;
        Vector                doubleTouchPos;
        Vector                doubleTouchVel;
    
        int doubleTouchStep;                 // the step the doubleTouch is in
        string      currentTask;

        BufferedPort<Bottle>  fgtTrackerPort;           
        Bottle               *fgtTrackerBottle;
        Vector                fgtTrackerPos;
        Vector                fgtTrackerVel;

       //The generic objects and sensManager ports are designed to handle multiple objects. For that reason, velocity estimation is not supported (keeping track of object IDs would be otherwise needed).        
        BufferedPort<Bottle>  genericObjectsPort;
        Bottle               *genericObjectsBottle;
        //collaboration with Jordi Ysard, UPF - WYSIWYD. Will be reading objects with "valence" - e.g. threat
        BufferedPort<Bottle>  sensManagerPort;
        Bottle               *sensManagerBottle;
       
    // Velocity Estimators (using adaptive window linear fitting)
        AWLinEstimator       *linEst_optFlow;
        AWLinEstimator       *linEst_pf3dTracker;
        AWLinEstimator       *linEst_doubleTouch;
        AWLinEstimator       *linEst_fgtTracker;

    // Drivers and Interfaces
        // Right arm
        PolyDriver       ddR; // right arm device driver
        
        IEncoders         *iencsR;
        yarp::sig::Vector *encsR;
        iCubArm           *armR;
        int                jntsR;

        // Left Arm
        PolyDriver       ddL; // left arm  device driver

        IEncoders         *iencsL;
        yarp::sig::Vector *encsL;
        iCubArm           *armL;
        int                jntsL;

        // Gaze Controller
        PolyDriver       ddG; // gaze  controller  driver
        IGazeControl    *igaze;
        int contextGaze;

    double timeNow;

    Port outPortGui;
    Port outPortEvents;

    /**
    * Handles the iCubGui, by drawing the perceived objects on the screen.
    **/
    void sendGuiEvents();

    /**
    * Deletes the objects from the gui.
    **/
    void deleteGuiEvents();

    /**
    * Prints a message according to the verbosity level:
    * @param l is the level of verbosity: if level > verbosity, something is printed
    * @param f is the text. Please use c standard (like printf)
    **/
    int printMessage(const int l, const char *f, ...) const;

public:
    // CONSTRUCTOR
    vtWThread(int _rate, const string &_name, const string &_robot, int _v, const ResourceFinder &_moduleRF);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
    // SAVE TAXELS TO FILE
    // bool save();
};

#endif

// empty line to make gcc happy
