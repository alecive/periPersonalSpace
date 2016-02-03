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
/**
\defgroup skinEventsAggregationModule skinEventsAggregationModule

@ingroup periPersonalSpace

Aggregates skinContact events (aka skin_events) - into one aggregated output per skin part with average location, normal and magnitude. 

Date first release: 01/02/2016

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
Aggregates skinContact events (aka skin_events) - into one aggregated output per skin part with average location, normal and magnitude as extracted from the skin positions files. Maximum 1 vector per skin part; format: (skinPart_s x y z o1 o2 o3 magnitude) (...) - for a maximum of the number of skin parts active

\section lib_sec Libraries 
YARP, ICUB libraries 

\section parameters_sec Parameters

--context       \e path
- Where to find the called resource.

--from          \e from
- The name of the .ini file with the configuration parameters.

--name          \e name
- The name of the module (default skinEventsAggregation).

--robot         \e rob
- The name of the robot (either "icub" or "icub"). Default icubSim.

--rate          \e rate
- The period used by the thread. Default 100ms, i.e. 10 Hz.

--verbosity  \e verb
- Verbosity level (default 0). The higher is the verbosity, the more
  information is printed out.

--type \e type
- Type of selection of contacts - e.g. random.


\section portsc_sec Ports Created
- <i> /<name>/skin_events:i </i> gets the skin events either from icub or icubSim

- <i> /<name>/skin_events:o </i> it sends out the aggegated skin events.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 12.04).

\author: Matej Hoffmann
*/ 

#include <yarp/os/all.h>
#include <string> 

#include <iCub/skinDynLib/skinContact.h>
#include <iCub/skinDynLib/skinContactList.h>

#include "skinEventsAggregThread.h"

using namespace yarp;
using namespace yarp::os;
using namespace std;

using namespace iCub::skinDynLib;

/**
* \ingroup skinEventsAggregationModule
*
* The module that achieves the skin events aggregation.
*  
*/
class skinEventsAggregator: public RFModule 
{
private:
    skinEventsAggregThread *skinEvAggThrd;
  
    string robot;
    string name;
    int verbosity;
    int threadPeriod;
    string type;    
       
    vector<SkinPart> activeSkinPartsNamesVector;
    map<SkinPart,string> skinPartsPositionsFilePaths;
 

public:
    skinEventsAggregator()
    {
        skinEvAggThrd = 0;
    }

    bool configure(ResourceFinder &rf)
    {
        
        name = "skinEventsAggregation";
        robot =  "icubSim";
        threadPeriod = 20; //period of the virtContactGenThread in ms
        verbosity = 0;
        
        SkinPart partOfSkin;
        //******************************************************
        //********************** CONFIGS ***********************

        //******************* GENERAL GROUP ******************
        Bottle &bGeneral=rf.findGroup("general");
        bGeneral.setMonitor(rf.getMonitor());  
        
            //******************* NAME ******************
            if (bGeneral.check("name"))
            {
                name = bGeneral.find("name").asString();
                yInfo("Module name set to %s", name.c_str());
            }
            else yInfo("Module name set to default, i.e. %s", name.c_str());
            setName(name.c_str());

            //******************* ROBOT ******************
            if (bGeneral.check("robot"))
            {
                robot = bGeneral.find("robot").asString();
                yInfo("Robot is: %s", robot.c_str());
            }
            else yInfo("Could not find robot option in the config file; using %s as default",robot.c_str());

            //****************** rate ******************
            if (bGeneral.check("rate"))
            {
                threadPeriod = bGeneral.find("rate").asInt();
                yInfo("skinEventsAggregThread rateThread working at %i ms.",threadPeriod);
            }
            else yInfo("Could not find rate in the config file; using %i ms as default period",threadPeriod);
           
            //******************* VERBOSE ******************
            if (bGeneral.check("verbosity"))
            {
                verbosity = bGeneral.find("verbosity").asInt();
                yInfo("verbosity set to %i", verbosity);
            }
            else yInfo("Could not find verbosity option in the config file; using %i as default",verbosity);  
            
           
           
        //*************** SKIN PARTS FILENAMES FROM SKIN MANAGER INI FILES ****************  
        
         ResourceFinder skinRF;
         int partNum;
         skinRF.setVerbose(false);
         skinRF.setDefaultContext("skinGui");                //overridden by --context parameter
         skinRF.setDefaultConfigFile("skinManAll.ini"); //overridden by --from parameter
         skinRF.configure(0,NULL);

         Bottle &skinEventsConf = skinRF.findGroup("SKIN_EVENTS");
         if(!skinEventsConf.isNull())
         {
            yInfo("SKIN_EVENTS section found\n");
            if(skinEventsConf.check("skinParts"))
            {
                Bottle* skinPartList = skinEventsConf.find("skinParts").asList();
                partNum=skinPartList->size();
            }
            if(skinEventsConf.check("taxelPositionFiles"))
            {
                Bottle *taxelPosFiles = skinEventsConf.find("taxelPositionFiles").asList();
                for(int i=0;i<partNum;i++)     // all of the skinparts
                {
                    string taxelPosFile = taxelPosFiles->get(i).asString().c_str();
                    string filePath(skinRF.findFile(taxelPosFile.c_str()));
                    if (filePath!="")
                    {
                        if(verbosity>0) yInfo("[skin_event] filePath [%i in bottle] %s; setting under %s in skinPartsPositionsFilePath.\n",i,filePath.c_str(),SkinPart_s[i+1].c_str());
                        skinPartsPositionsFilePaths[(SkinPart)(i+1)] = filePath;    //! Importantly, this is relying on the fact that the skin parts are in the 
                        //right order in the .ini file, matching with SkinPart enum, and starting with 1 for left hand
                    }
                }
            }
            else
            {
                yError(" No skin's configuration files found.");
                return 0;
            }
        }
           
        //******************************************************
        //*********************** THREAD **********************
        skinEvAggThrd = new skinEventsAggregThread(threadPeriod,name,robot,verbosity,skinPartsPositionsFilePaths);
        if (!skinEvAggThrd -> start())
        {
              delete skinEvAggThrd;
              skinEvAggThrd = 0;
              yError("skinEventsAggregThread wasn't instantiated!!");
                    return false;
        }
        yInfo("skinEventsAggregThread instantiated...");

        return true;
    }

    bool close()
    {
        yInfo("skinEventsAggregation: Stopping thread..");
        if (skinEvAggThrd)
        {
            skinEvAggThrd -> stop();
            delete skinEvAggThrd;
            skinEvAggThrd =  0;
        }
        
        return true;
    }

    double getPeriod()
    {
        return 1.0;
    }

    bool updateModule()
    {
        return true;
    }
};


/**
* Main function.
*/
int main(int argc, char * argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("periPersonalSpace");
    rf.setDefaultConfigFile("skinEventsAggregation.ini");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {   
        yInfo(""); 
        yInfo("Options:");
        yInfo("");
        yInfo("  --context    path:  where to find the called resource");
        yInfo("  --from       from:  the name of the .ini file.");
        yInfo("  --general::name      name:  the name of the module (default virtualContactGeneration).");
        yInfo("  --general::robot     robot: the name of the robot. Default icubSim.");
        yInfo("  --general::rate      rate:  the period used by the thread. Default 100ms.");
        yInfo("  --general::verbosity int:   verbosity level (default 0).");
        yInfo("");
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    skinEventsAggregator sEA;
    return sEA.runModule(rf);
}
// empty line to make gcc happy
