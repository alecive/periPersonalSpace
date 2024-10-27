/* VISUO TACTILE RECEPTIVE FIELDS v. 1.0
 * Copyright (C) 2013 RobotCub Consortium
 * Author:  Alessandro Roncone & Matej Hoffmann
 * email:    alessandro.roncone@yale.edu, matej.hoffmann@iit.it
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
\defgroup visuoTactileRFModule visuoTactileRFModule

@ingroup periPersonalSpace

A module that handles the visuoTactile Receptive Fields

Date first release: 23/10/2013

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This is a module for implementing the VisuoTactile ReceptiveFields on the iCub.

\section lib_sec Libraries 
YARP, ICUB libraries and OPENCV

\section parameters_sec Parameters

--context       \e path
- Where to find the called resource.

--from          \e from
- The name of the .ini file with the configuration parameters (default visuoTactileRF.ini).

--name          \e name
- The name of the module (default visuoTactileRF).

--robot         \e rob
- The name of the robot (either "icub" or "icub"). Default icub.

--arm_version   \e ver
- Version of the arm kinematics (default: 1.0 for robot==icubSim; 2.0 for robot==icub). 

--rate          \e rate
- The period used by the thread. Default 100ms.

--taxelsFile    \e file
- The name of the file from which a learned peripersonal space representation will be loaded from.
  Default 'taxels1D.ini' if modality is 1D, otherwise 'taxels2D.ini'. 
  Saving will by default be into [file]_out.ini
  
--verbosity     \e verb
- verbosity level

\section portsc_sec Ports Created
- <i> /<name>/contacts:i </i> it reads the skinContacts from the skinManager.

- <i> /<name>/records:o </i> it prints out some data about the state of the
  recording.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section tested_os_sec Tested OS
Linux (Ubuntu 12.04, Ubuntu 14.04, Debian Squeeze, Debian Wheezy).

\authors: Alessandro Roncone, Matej Hoffmann
*/ 

#include <yarp/os/all.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include <iostream>
#include <string> 

#include "vtRFThread.h"


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace std;

/**
* \ingroup visuoTactileRFModule
*
* The module that achieves the visuoTactileRF task.
*  
*/
class visuoTactileRF: public RFModule 
{
private:
    vtRFThread *vtRFThrd;

    RpcClient             rpcClnt;
    RpcServer             rpcSrvr;

    string robot,name,modality;
    int verbosity,rate;
    double arm_version;

public:
    visuoTactileRF()
    {
        vtRFThrd    = 0;
    }

    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack =Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            switch (command.get(0).asVocab())
            {
                case createVocab('s','a','v','e'):
                {
                    int res=Vocab::encode("saved");
                    string fileName = vtRFThrd -> save();

                    if (fileName=="")
                    {
                        reply.addVocab(nack);
                    }
                    else
                    {
                        reply.addVocab(ack);
                        reply.addString(fileName.c_str());
                    }
                    
                    reply.addVocab(res);
                    return true;
                }
                case createVocab('l','o','a','d'):
                {
                    int res=Vocab::encode("loaded");
                    string fileName=vtRFThrd -> load();

                    if (fileName=="")
                    {
                        reply.addVocab(nack);
                    }
                    else
                    {
                        reply.addVocab(ack);
                        reply.addString(fileName.c_str());
                    }
                    
                    reply.addVocab(res);
                    return true;
                }
                case createVocab('r','e','s','e'):
                {
                    vtRFThrd -> resetParzenWindows();
                    reply.addVocab(ack);
                    return true;
                }
                case createVocab('s','t','o','p'):
                {
                    vtRFThrd -> stopLearning();
                    reply.addVocab(ack);
                    return true;
                }
               case createVocab('r','e','s','t'):
                {
                    vtRFThrd -> restoreLearning();
                    reply.addVocab(ack);
                    return true;
                }     
                //-----------------
                default:
                    return RFModule::respond(command,reply);
            }
        }

        reply.addVocab(nack);
        return true;
    }

    bool configure(ResourceFinder &rf)
    {
        name     = "visuoTactileRF";
        robot    = "icub";
        modality = "1D";

        verbosity  = 0;     // verbosity
        rate       = 20;    // rate of the vtRFThread

        //******************************************************
        //********************** CONFIGS ***********************

        //******************* NAME ******************
            if (rf.check("name"))
            {
                name = rf.find("name").asString();
                yInfo("Module name set to %s", name.c_str());
            }
            else yInfo("Module name set to default, i.e. %s", name.c_str());
            setName(name.c_str());

        //******************* ROBOT ******************
            if (rf.check("robot"))
            {
                robot = rf.find("robot").asString();
                yInfo("Robot is %s", robot.c_str());
            }
            else yInfo("Could not find robot option in the config file; using %s as default",robot.c_str());
            
        //******************* ARM VERSION ******************
            arm_version = 2.0;
            if (rf.check("arm_version"))
            {
                arm_version = rf.find("arm_version").asDouble();
                yInfo("Setting arm version to %f", arm_version);
            }
            else yInfo("Could not find arm_version option in the config file; using %f as default for this robot type",arm_version);    

        //***************** MODALITY *****************
            if (rf.check("modality"))
            {
                modality = rf.find("modality").asString();
                yInfo("modality is  %s", modality.c_str());
            }
            else yInfo("Could not find modality option in the config file; using %s as default",modality.c_str());

        //******************* VERBOSE ******************
            if (rf.check("verbosity"))
            {
                verbosity = rf.find("verbosity").asInt();
                yInfo("vtRFThread verbosity set to %i", verbosity);
            }
            else yInfo("Could not find verbosity option in config file; using %i as default",verbosity);

        //****************** rate ******************
            if (rf.check("rate"))
            {
                rate = rf.find("rate").asInt();
                yInfo("vtRFThread working at  %i ms", rate);
            }
            else yInfo("Could not find rate in the config file; using %i ms as default",rate);

        //************* skinManager Resource finder **************
            ResourceFinder skinRF;
            skinRF.setDefaultContext("skinGui");                //overridden by --context parameter
            skinRF.setDefaultConfigFile("skinManAll.ini"); //overridden by --from parameter
            skinRF.configure(0,NULL);
            
            vector<string> filenames;
            //int partNum=4;

            Bottle &skinEventsConf = skinRF.findGroup("SKIN_EVENTS");
            if(!skinEventsConf.isNull())
            {
                yInfo("SKIN_EVENTS section found\n");

                /*if(skinEventsConf.check("skinParts"))
                {
                    Bottle* skinPartList = skinEventsConf.find("skinParts").asList();
                    partNum=skinPartList->size();
                }*/
                // the code below relies on a fixed ordre of taxel pos files in skinManAll.ini
                if(skinEventsConf.check("taxelPositionFiles"))
                {
                    Bottle *taxelPosFiles = skinEventsConf.find("taxelPositionFiles").asList();
                    for (int i = 0; i < 7; i++)
                    {
                        string taxelPosFile = taxelPosFiles->get(i).asString();
                        string filePath(skinRF.findFile(taxelPosFile));
                        if (!filePath.empty())
                        {
                            yInfo("[visuoTactileRF] filePath: %s\n",filePath.c_str());
                            filenames.push_back(filePath);
                        }
                        taxelPosFile.clear(); filePath.clear();
                    }
                }
            }
            else
            {
                yError(" No skin configuration files found.");
                return 0;
            }

        //*************** eyes' Resource finder ****************
            ResourceFinder gazeRF;
            gazeRF.setDefaultContext("iKinGazeCtrl");
            robot=="icub"?gazeRF.setDefaultConfigFile("config.ini"):gazeRF.setDefaultConfigFile("configSim.ini");
            gazeRF.configure(0,NULL);
            double head_version=gazeRF.check("head_version",Value(1.0)).asDouble();

            ResourceFinder eyeAlignRF;

            Bottle &camerasGroup = gazeRF.findGroup("cameras");

            if(!camerasGroup.isNull())
            {
                camerasGroup.check("context")?
                eyeAlignRF.setDefaultContext(camerasGroup.find("context").asString().c_str()):
                eyeAlignRF.setDefaultContext(gazeRF.getContext().c_str());
                eyeAlignRF.setDefaultConfigFile(camerasGroup.find("file").asString().c_str()); 
                eyeAlignRF.configure(0,NULL); 
            }
            else
            {
                yWarning("Did not find camera calibration group into iKinGazeCtrl ResourceFinder!");        
            }

        //******************************************************
        //*********************** THREAD **********************
            if( filenames.size() > 0 )
            {
                vtRFThrd = new vtRFThread(rate, name, robot, modality, verbosity, rf,
                                          filenames, head_version, arm_version, eyeAlignRF);
                if (!vtRFThrd -> start())
                {
                    delete vtRFThrd;
                    vtRFThrd = 0;
                    yError("vtRFThread wasn't instantiated!!");
                    return false;
                }
                yInfo("VISUO TACTILE RECEPTIVE FIELDS: vtRFThread instantiated...");
            }
            else {
                vtRFThrd = 0;
                yError("vtRFThread wasn't instantiated. No filenames have been given!");
                return false;
            }

        //******************************************************
        //************************ PORTS ***********************
            rpcClnt.open(("/"+name+"/rpc:o").c_str());
            rpcSrvr.open(("/"+name+"/rpc:i").c_str());
            attach(rpcSrvr);

        return true;
    }

    bool close()
    {
        yInfo("VISUO TACTILE RECEPTIVE FIELDS: Stopping thread..");
        if (vtRFThrd)
        {
            vtRFThrd->stop();
            delete vtRFThrd;
            vtRFThrd=0;
        }
        return true;
    }

    double getPeriod()
    {
        return 0.05;
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
    
    ResourceFinder moduleRF;
    moduleRF.setDefaultContext("periPersonalSpace");
    moduleRF.setDefaultConfigFile("visuoTactileRF.ini");
    moduleRF.configure(argc,argv);

    if (moduleRF.check("help"))
    {    
        yInfo(" ");
        yInfo("Options:");
        yInfo("  --context    path:   where to find the called resource (default periPersonalSpace).");
        yInfo("  --from       from:   the name of the .ini file (default visuoTactileRF.ini).");
        yInfo("  --name       name:   the name of the module (default visuoTactileRF).");
        yInfo("  --robot      robot:  the name of the robot. Default icub.");
        yInfo("  --arm_version   arm_version:  arm kinematics version. Default 2.0 for icub, 1.0 for icubSim.");
        yInfo("  --rate       rate:   the period used by the thread. Default 20ms.");
        yInfo("  --verbosity  int:    verbosity level (default 0).");
        yInfo("  --modality   string: which modality to use (either 1D or 2D, default 1D).");
        yInfo("  --taxelsFile string: the file from which load and save taxels' PPS representations. Defaults:");
        yInfo("                       'taxels1D.ini' if modality==1D");
        yInfo("                       'taxels2D.ini' if modality==2D");
        yInfo("  --rightForeArm, --rightHand, --leftForeArm, --leftHand     flag: flag(s) to call if the module");
        yInfo("                       has to be run with either one of the 4 skin parts available.");
        yInfo("                       Not using any of them is equal to call all of them at the same time.");
        yInfo(" ");
        return 0;
    }
    
    if (!yarp.checkNetwork())
    {
        printf("No Network!!!\n");
        return -1;
    }

    visuoTactileRF visTacRF;
    return visTacRF.runModule(moduleRF);
}

// empty line to make gcc happy
