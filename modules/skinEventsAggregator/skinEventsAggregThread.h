/*
 * Copyright: (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Matej Hoffmann
 * email:  matej.hoffmann@iit.it
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
*/

#ifndef __SKINEVENTSAGGREGTHREAD_H__
#define __SKINEVENTSAGGREGTHREAD_H__

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <stdarg.h>
#include <string> 
#include <iostream>
#include <fstream>
#include <sstream>

#include <yarp/os/Time.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Stamp.h>

#include <yarp/math/Math.h>

#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinPart.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <iCub/periPersonalSpace/utils.h>

using namespace std;

class skinEventsAggregThread: public yarp::os::PeriodicThread
{
private:
    /***************************************************************************/
    // EXTERNAL VARIABLES: change them from command line or through .ini file
    // Flag that manages verbosity (v=1 -> more text printed out; v=2 -> even more text):
    int verbosity;
    // Name of the module (to change port names accordingly):
    string name;
    // Name of the robot (to address the module toward icub or icubSim):
    string robot;
    // Resource finder used to find for files and configurations:
    yarp::os::ResourceFinder* rf;
    //the period used by the thread. 
    int threadPeriod; 
       
    /***************************************************************************/
    // INTERNAL VARIABLES
    
    // Stamp for the setEnvelope for the ports
    yarp::os::Stamp ts;
    
    yarp::os::BufferedPort<iCub::skinDynLib::skinContactList> skinEventsPortIn;  // input from the skinManager - skin_events port
    yarp::os::BufferedPort<yarp::os::Bottle> skinEvAggregPortOut;                // output for the transformed skin events
   
    double SKIN_ACTIVATION_MAX;
 
    int getIndexOfBiggestContactInList(iCub::skinDynLib::skinContactList &sCL);
    
    /**
    * Prints a message according to the verbosity level:
    * @param l will be checked against the global var verbosity: if verbosity >= l, something is printed
    * @param f is the text. Please use c standard (like printf)
    */
    int printMessage(const int l, const char *f, ...) const;

public:
    // CONSTRUCTOR
    skinEventsAggregThread(int _rate, const string &_name, const string &_robot, int _v);
    // INIT
    virtual bool threadInit();
    // RUN
    virtual void run();
    // RELEASE
    virtual void threadRelease();
 

};

#endif
