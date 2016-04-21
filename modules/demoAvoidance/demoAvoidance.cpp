/* DOUBLE TOUCH v. 0.2
 * Copyright (C) 2013 RobotCub Consortium
 * Author:  Alessandro Roncone
 * email:   alessandro.roncone@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/"+robot+"/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <stdio.h>
#include <string>
#include <cctype>
#include <algorithm>
#include <map>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/math.h>
#include <iCub/skinDynLib/common.h>

#define PPS_AVOIDANCE_RADIUS        0.2     // [m]
#define PPS_AVOIDANCE_VELOCITY      0.10    // [m/s]
#define PPS_AVOIDANCE_PERSISTENCE   0.3     // [s]
#define PPS_AVOIDANCE_TIMEOUT       1.0     // [s]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


//*************************************
class Avoidance: public RFModule,
                 public PortReader
{
protected:
    PolyDriver driverCartL,driverCartR;
    PolyDriver driverJointL,driverJointR;
    int contextL,contextR;
    Port dataPort;
    double motionGain;

    bool useLeftArm;
    bool useRightArm;

    RpcServer          rpcSrvr;

    Mutex mutex;
    struct Data {
        ICartesianControl *iarm;
        Vector point, dir;
        Vector home_x, home_o;
        double persistence, timeout;
        Data(): point(3,0.0), dir(3,0.0),
                home_x(3,0.0), home_o(4,0.0),
                persistence(0.0), timeout(-1.0) { }
    };
    map<string,Data> data;
    
    //********************************************
    bool read(ConnectionReader &connection)
    {
        Bottle in;
        in.read(connection);

        if (in.size()>=1)
        {
            Bottle input;
            input=*(in.get(0).asList());

            if (input.size()>=7)
            {
                yDebug("[demoAvoidance] got command (%s)",input.toString().c_str());
                string arm=iCub::skinDynLib::SkinPart_s[input.get(0).asInt()];

                if (arm == "r_hand" || arm == "r_forearm")
                {
                    arm = "right";
                }
                else if (arm == "l_hand" || arm == "l_forearm")
                {
                    arm = "left";
                }
                
                transform(arm.begin(),arm.end(),arm.begin(),::tolower);
                mutex.lock();
                map<string,Data>::iterator it=data.find(arm);
                if (it!=data.end())
                {
                    Data &d=it->second;
                    d.point[0]=input.get(7).asDouble();
                    d.point[1]=input.get(8).asDouble();
                    d.point[2]=input.get(9).asDouble();
                    d.dir[0]=input.get(10).asDouble();
                    d.dir[1]=input.get(11).asDouble();
                    d.dir[2]=input.get(12).asDouble();
                    d.persistence=PPS_AVOIDANCE_PERSISTENCE;
                    d.timeout=PPS_AVOIDANCE_TIMEOUT;
                }
                mutex.unlock();
            }
        }

        return true;
    }

    //********************************************
    void manageArm(Data &data)
    {
        if (data.persistence>0.0)
        {
            Vector x,o;
            data.iarm->getPose(x,o);
            if (norm(x-data.home_x)>PPS_AVOIDANCE_RADIUS)
                data.iarm->stopControl();
            else
                data.iarm->setTaskVelocities((motionGain*PPS_AVOIDANCE_VELOCITY/norm(data.dir))*data.dir,Vector(4,0.0)); 

            data.persistence=std::max(data.persistence-getPeriod(),0.0);
            if (data.persistence==0.0)
                data.iarm->stopControl();
        }
        else if (data.timeout>0.0)
        {
            data.timeout=std::max(data.timeout-getPeriod(),0.0);
        }
        else if (data.timeout==0.0)
        {
            data.iarm->goToPose(data.home_x,data.home_o);
            data.timeout=-1.0;
        }
    }
    
public:
    bool respond(const Bottle &command, Bottle &reply)
    {
        int ack =Vocab::encode("ack");
        int nack=Vocab::encode("nack");

        if (command.size()>0)
        {
            if (command.get(0).asString() == "get")
            {
                if (command.get(1).asString() == "motionGain")
                {
                    reply.addVocab(ack);
                    reply.addDouble(motionGain);
                }
                else
                {
                    reply.addVocab(nack);
                }
            }
            else if (command.get(0).asString() == "set")
            {
                if (command.get(1).asString() == "motionGain")
                {
                    reply.addVocab(ack);
                    motionGain = command.get(2).asDouble();
                    reply.addDouble(motionGain);
                }
                else if (command.get(1).asString() == "behavior")
                {
                    if (command.get(2).asString() == "avoidance")
                    {
                        reply.addVocab(ack);
                        motionGain = -1.0;
                        reply.addDouble(motionGain);
                    }
                    else if (command.get(2).asString() == "catching")
                    {
                        reply.addVocab(ack);
                        motionGain = 1.0;
                        reply.addDouble(motionGain);
                    }
                    else
                    {
                        reply.addVocab(nack);
                    }
                }
                else
                {
                    reply.addVocab(nack);
                }
            }
        }

        return true;
    }

    //********************************************
    bool configure(ResourceFinder &rf)
    {
        if (rf.check("noLeftArm") && rf.check("noRightArm"))
        {
            yError("[demoAvoidance] no arm has been selected. Closing.");
            return false;
        }

        useLeftArm  = false;
        useRightArm = false;

        string  name=rf.check("name",Value("avoidance")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();
        motionGain = -1.0;
        if (rf.check("catching"))
        {
            motionGain = 1.0;
        }
        if (motionGain!=-1.0)
        {
            yWarning("[demoAvoidance] motionGain set to catching");
        }

        bool autoConnect=rf.check("autoConnect");
        if (autoConnect)
        {
            yWarning("[demoAvoidance] Autoconnect mode set to ON");
        }

        bool stiff=rf.check("stiff");
        if (stiff)
        {
            yInfo("[demoAvoidance] Stiff Mode enabled.");
        }

        Matrix R(4,4);
        R(0,0)=-1.0; R(2,1)=-1.0; R(1,2)=-1.0; R(3,3)=1.0;

        if (!rf.check("noLeftArm"))
        {
            useLeftArm=true;

            data["left"]=Data();
            data["left"].home_x[0]=-0.30;
            data["left"].home_x[1]=-0.20;
            data["left"].home_x[2]=+0.05;
            data["left"].home_o=dcm2axis(R);

            Property optionCartL;
            optionCartL.put("device","cartesiancontrollerclient");
            optionCartL.put("remote","/"+robot+"/cartesianController/left_arm");
            optionCartL.put("local",("/"+name+"/cart/left_arm").c_str());
            if (!driverCartL.open(optionCartL))
            {
                close();
                return false;
            }

            Property optionJointL;
            optionJointL.put("device","remote_controlboard");
            optionJointL.put("remote","/"+robot+"/left_arm");
            optionJointL.put("local",("/"+name+"/joint/left_arm").c_str());
            if (!driverJointL.open(optionJointL))
            {
                close();
                return false;
            }

            driverCartL.view(data["left"].iarm);
            data["left"].iarm->storeContext(&contextL);

            Vector dof;
            data["left"].iarm->getDOF(dof);
            dof=0.0; dof[3]=dof[4]=dof[5]=dof[6]=1.0;
            data["left"].iarm->setDOF(dof,dof);
            data["left"].iarm->setTrajTime(0.9);

            data["left"].iarm->goToPoseSync(data["left"].home_x,data["left"].home_o);
            data["left"].iarm->waitMotionDone();

            IInteractionMode  *imode; driverJointL.view(imode);
            IImpedanceControl *iimp;  driverJointL.view(iimp);

            if (!stiff)
            {
                imode->setInteractionMode(0,VOCAB_IM_COMPLIANT); iimp->setImpedance(0,0.4,0.03); 
                imode->setInteractionMode(1,VOCAB_IM_COMPLIANT); iimp->setImpedance(1,0.4,0.03);
                imode->setInteractionMode(2,VOCAB_IM_COMPLIANT); iimp->setImpedance(2,0.4,0.03);
                imode->setInteractionMode(3,VOCAB_IM_COMPLIANT); iimp->setImpedance(3,0.2,0.01);
                imode->setInteractionMode(4,VOCAB_IM_COMPLIANT); iimp->setImpedance(4,0.2,0.0);
            }
        }

        if (!rf.check("noRightArm"))
        {
            useRightArm = true;

            data["right"]=Data();
            data["right"].home_x[0]=-0.30;
            data["right"].home_x[1]=+0.20;
            data["right"].home_x[2]=+0.05;
            data["right"].home_o=dcm2axis(R);

            Property optionCartR;
            optionCartR.put("device","cartesiancontrollerclient");
            optionCartR.put("remote","/"+robot+"/cartesianController/right_arm");
            optionCartR.put("local",("/"+name+"/cart/right_arm").c_str());
            if (!driverCartR.open(optionCartR))
            {
                close();
                return false;
            }

            Property optionJointR;
            optionJointR.put("device","remote_controlboard");
            optionJointR.put("remote","/"+robot+"/right_arm");
            optionJointR.put("local",("/"+name+"/joint/right_arm").c_str());
            if (!driverJointR.open(optionJointR))
            {
                close();
                return false;
            }

            driverCartR.view(data["right"].iarm);
            data["right"].iarm->storeContext(&contextR);

            Vector dof;
            data["right"].iarm->getDOF(dof);
            dof=0.0; dof[3]=dof[4]=dof[5]=dof[6]=1.0;
            data["right"].iarm->setDOF(dof,dof);
            data["right"].iarm->setTrajTime(0.9);

            data["right"].iarm->goToPoseSync(data["right"].home_x,data["right"].home_o);
            data["right"].iarm->waitMotionDone();


            IInteractionMode  *imode; driverJointR.view(imode);
            IImpedanceControl *iimp;  driverJointR.view(iimp);

            if (!stiff)
            {
                imode->setInteractionMode(0,VOCAB_IM_COMPLIANT); iimp->setImpedance(0,0.4,0.03); 
                imode->setInteractionMode(1,VOCAB_IM_COMPLIANT); iimp->setImpedance(1,0.4,0.03);
                imode->setInteractionMode(2,VOCAB_IM_COMPLIANT); iimp->setImpedance(2,0.4,0.03);
                imode->setInteractionMode(3,VOCAB_IM_COMPLIANT); iimp->setImpedance(3,0.2,0.01);
                imode->setInteractionMode(4,VOCAB_IM_COMPLIANT); iimp->setImpedance(4,0.2,0.0);
            }
        }

        dataPort.open(("/"+name+"/data:i").c_str());
        dataPort.setReader(*this);

        if (autoConnect)
        {
            Network::connect("/visuoTactileRF/skin_events:o",("/"+name+"/data:i").c_str());
        }

        rpcSrvr.open(("/"+name+"/rpc:i").c_str());
        attach(rpcSrvr);
        return true; 
    }

    //********************************************
    bool updateModule()
    {
        mutex.lock();
        if (useLeftArm)
        {
            manageArm(data["left"]);
        }
        
        if (useRightArm)
        {
            manageArm(data["right"]);
        }
        mutex.unlock(); 
        return true;
    }

    //********************************************
    double getPeriod()
    {
        return 0.05;
    }

    //********************************************
    bool close()
    {
        yInfo("[demoAvoidance] Closing module..");

        dataPort.close();

        if (useLeftArm)
        {
            if (driverCartL.isValid())
            {
                data["left"].iarm->stopControl();
                data["left"].iarm->restoreContext(contextL);
                driverCartL.close(); 
            }

            if (driverJointL.isValid())
            {
                IInteractionMode *imode;
                driverJointL.view(imode);
                for (int j=0; j<5; j++)
                    imode->setInteractionMode(j,VOCAB_IM_STIFF);

                driverJointL.close();
            }
        }

        if (useRightArm)
        {
            if (driverCartR.isValid())
            {
                data["right"].iarm->stopControl();
                data["right"].iarm->restoreContext(contextR);
                driverCartR.close();
            }

            if (driverJointR.isValid())
            {
                IInteractionMode *imode;
                driverJointR.view(imode);
                for (int j=0; j<5; j++)
                    imode->setInteractionMode(j,VOCAB_IM_STIFF);

                driverJointR.close();
            }
        }

        return true;
    }
};


//********************************************
int main(int argc, char * argv[])
{
    Network yarp;

    ResourceFinder moduleRF;
    moduleRF.setVerbose(false);
    moduleRF.setDefaultContext("periPersonalSpace");
    moduleRF.setDefaultConfigFile("demoAvoidance.ini");
    moduleRF.configure(argc,argv);

    if (moduleRF.check("help"))
    {    
        yInfo("");
        yInfo("Options:");
        yInfo("   --context     path:  where to find the called resource (default periPersonalSpace).");
        yInfo("   --from        from:  the name of the .ini file (default demoAvoidance.ini).");
        yInfo("   --name        name:  the name of the module (default avoidance).");
        yInfo("   --autoConnect flag:  if to auto connect the ports or not. Default no.");
        yInfo("   --catching    flag:  if enabled, the robot will catch the target instead of avoiding it.");
        yInfo("   --stiff       flag:  if enabled, the robot will perform movements in stiff mode instead of compliant.");
        yInfo("   --noLeftArm   flag:  if enabled, the robot will perform movements without the left arm.");
        yInfo("   --noRightArm  flag:  if enabled, the robot will perform movements without the rihgt arm.");
        yInfo("");
        return 0;
    }

    if (!yarp.checkNetwork())
    {
        yError("No Network!!!");
        return -1;
    }

    Avoidance module;
    return module.runModule(moduleRF);
}


