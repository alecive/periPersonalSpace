#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <iomanip>

#include "vtRFThread.h"

#define RADIUS              2 // Radius in px of every taxel (in the images)
#define SKIN_THRES	        7 // Threshold with which a contact is detected
#define RESP_GAIN_FOR_SKINGUI 100 //To amplify PPS activations from <0,1> to <0,100>
#define PPS_AGGREG_ACT_THRESHOLD 0.2 //Threshold for aggregated events per skin part

IncomingEvent eventFromBottle(const Bottle &b)
{
    IncomingEvent ie;
    ie.fromBottle(b);
    return ie;
};

vtRFThread::vtRFThread(int _rate, const string &_name, const string &_robot, const string &_modality,
                       int _v, const ResourceFinder &_moduleRF, vector<string> _fnames,
                       double _hV, double _armV, const ResourceFinder &_eyeCalibRF) :
                       PeriodicThread((double)_rate/1000.0), name(_name), robot(_robot), modality(_modality),
                       verbosity(_v), filenames(_fnames)
{
    //******************* PORTS ******************
        imagePortInR  = new BufferedPort<ImageOf<PixelRgb> >;
        imagePortInL  = new BufferedPort<ImageOf<PixelRgb> >;
        dTPort        = new BufferedPort<Bottle>;
        stressPort    = new BufferedPort<Bottle>;
        eventsPort    = new BufferedPort<Bottle>;
        skinPortIn    = new BufferedPort<iCub::skinDynLib::skinContactList>;

    //******************* ARMS, EYEWRAPPERS ******************
        
        std::ostringstream strR;
        strR<<"right_v"<<_armV;
        std::string typeR = strR.str();
        armR = new iCubArm(typeR);
        
        std::ostringstream strL;
        strL<<"left_v"<<_armV;
        std::string typeL = strL.str();
        armL = new iCubArm(typeL);
        torso = new iCubTorso();
              
        eWR  = new eyeWrapper("right",_hV,_eyeCalibRF);
        eWL  = new eyeWrapper("left", _hV,_eyeCalibRF);

        rf = const_cast<ResourceFinder*>(&_moduleRF);

        eventsFlag   = true;
        learningFlag = false;

    //******************* PATH ******************
        path = rf->getHomeContextPath();
        path = path+"/";
        if (rf->check("taxelsFile"))
        {
            taxelsFile = rf -> find("taxelsFile").asString();
        }
        else
        {
            taxelsFile = "taxels"+modality+".ini";
            rf -> setDefault("taxelsFile",taxelsFile);
        }
        yInfo("Storing file set to: %s", (path+taxelsFile).c_str());
}

bool vtRFThread::threadInit()
{
    bool ok = true;

    imagePortInR        -> open(("/"+name+"/imageR:i"));
    imagePortInL        -> open(("/"+name+"/imageL:i"));
    imagePortOutR.open(("/"+name+"/imageR:o"));
    imagePortOutL.open(("/"+name+"/imageL:o"));
    dTPort              -> open(("/"+name+"/input:i"));
    stressPort          -> open(("/"+name+"/stress:i"));
    eventsPort          -> open(("/"+name+"/events:i"));
    skinGuiPortForearmL.open(("/"+name+"/skinGuiForearmL:o"));
    skinGuiPortForearmR.open(("/"+name+"/skinGuiForearmR:o"));
    skinGuiPortUpperarmL.open(("/"+name+"/skinGuiUpperarmL:o"));
    skinGuiPortUpperarmR.open(("/"+name+"/skinGuiUpperarmR:o"));
    skinGuiPortHandL.open(("/"+name+"/skinGuiHandL:o"));
    skinGuiPortHandR.open(("/"+name+"/skinGuiHandR:o"));
    skinGuiPortTorso.open(("/"+name+"/skinGuiTorso:o"));
    skinPortIn          -> open(("/"+name+"/skin_events:i"));
    ppsEventsPortOut.open(("/"+name+"/pps_events_aggreg:o"));
    dataDumperPortOut.open(("/"+name+"/dataDumper:o"));


    // Autoconnection is not recommended; the list below is not uptodate anymore
        // if (robot=="icub")
        // {
        //     Network::connect("/icub/camcalib/left/out",("/"+name+"/imageL:i"));
        //     Network::connect("/icub/camcalib/right/out",("/"+name+"/imageR:i"));
        // }
        // else
        // {
        //     Network::connect("/icubSim/cam/left",("/"+name+"/imageL:i"));
        //     Network::connect("/icubSim/cam/right",("/"+name+"/imageR:i"));
        // }
        // Network::connect(("/"+name+"/imageL:o"),"/vtRF/left");
        // Network::connect(("/"+name+"/imageR:o"),"/vtRF/right");
        // Network::connect("/doubleTouch/status:o",("/"+name+"/input:i"));
        // Network::connect("/visuoTactileWrapper/events:o",("/"+name+"/events:i"));
        // Network::connect(("/"+name+"/skinGuiForearmL:o"),"/skinGui/left_forearm_virtual:i");
        // Network::connect(("/"+name+"/skinGuiForearmR:o"),"/skinGui/right_forearm_virtual:i");
        // Network::connect(("/"+name+"/skinGuiHandL:o"),"/skinGui/left_hand_virtual:i");
        // Network::connect(("/"+name+"/skinGuiHandR:o"),"/skinGui/right_hand_virtual:i");
        // Network::connect("/skinManager/skin_events:o",("/"+name+"/skin_events:i"));

        ts.update();

        stress = 0.0;
        jntsT = 3; //nr torso joints
    /********** Open right arm interfaces (if they are needed) ***************/
        if (rf->check("rightHand") || rf->check("rightForeArm") ||
           (!rf->check("rightHand") && !rf->check("rightForeArm") && !rf->check("leftHand") && !rf->check("leftForeArm")))
        {
            for (int i = 0; i < jntsT; i++)
                armR->releaseLink(i); //torso will be enabled
            Property OptR;
            OptR.put("robot",  robot);
            OptR.put("part",   "right_arm");
            OptR.put("device", "remote_controlboard");
            OptR.put("remote",("/"+robot+"/right_arm"));
            OptR.put("local", ("/"+name +"/right_arm"));

            if (!ddR.open(OptR))
            {
                yError("[vtRFThread] : could not open right_arm PolyDriver!\n");
                return false;
            }
            ok = true;
            if (ddR.isValid())
            {
                ok = ok && ddR.view(iencsR);
            }
            if (!ok)
            {
                yError("[vtRFThread] Problems acquiring right_arm interfaces!!!!\n");
                return false;
            }
            iencsR->getAxes(&jntsR);
            encsR = new yarp::sig::Vector(jntsR,0.0); //should be 16 - arm + fingers
            jntsAR = 7; //nr. arm joints only - without fingers
            qR.resize(jntsAR,0.0); //current values of arm joints (should be 7)
            
        }

    /********** Open left arm interfaces (if they are needed) ****************/
        if (rf->check("leftHand") || rf->check("leftForeArm") ||
           (!rf->check("rightHand") && !rf->check("rightForeArm") && !rf->check("leftHand") && !rf->check("leftForeArm")))
        {
            for (int i = 0; i < jntsT; i++)
                armL->releaseLink(i); //torso will be enabled
            Property OptL;
            OptL.put("robot",  robot);
            OptL.put("part",   "left_arm");
            OptL.put("device", "remote_controlboard");
            OptL.put("remote",("/"+robot+"/left_arm"));
            OptL.put("local", ("/"+name +"/left_arm"));

            if (!ddL.open(OptL))
            {
                yError("[vtRFThread] : could not open left_arm PolyDriver!\n");
                return false;
            }
            ok = true;
            if (ddL.isValid())
            {
                ok = ok && ddL.view(iencsL);
            }
            if (!ok)
            {
                yError("[vtRFThread] Problems acquiring left_arm interfaces!!!!\n");
                return false;
            }
            iencsL->getAxes(&jntsL);
            encsL = new yarp::sig::Vector(jntsL,0.0); //should be 16 - arm + fingers
            jntsAL = 7; //nr. arm joints only - without fingers
            qL.resize(jntsAL,0.0); //current values of arm joints (should be 7)
            
        }

    /**************************/
        Property OptT;
        OptT.put("robot",  robot);
        OptT.put("part",   "torso");
        OptT.put("device", "remote_controlboard");
        OptT.put("remote",("/"+robot+"/torso"));
        OptT.put("local", ("/"+name +"/torso"));

        if (!ddT.open(OptT))
        {
            yError("[vtRFThread] Could not open torso PolyDriver!");
            return false;
        }
        ok = true;
        if (ddT.isValid())
        {
            ok = ok && ddT.view(iencsT);
        }
        if (!ok)
        {
            yError("[vtRFThread] Problems acquiring head interfaces!!!!");
            return false;
        }
        iencsT->getAxes(&jntsT);
        encsT = new yarp::sig::Vector(jntsT,0.0);
        qT.resize(jntsT,0.0); //current values of torso joints (3, in the order expected for iKin: yaw, roll, pitch)
        for (int i = 0; i < jntsT; i++)
            torso->releaseLink(i); //torso will be enabled

    /**************************/
        Property OptH;
        OptH.put("robot",  robot);
        OptH.put("part",   "head");
        OptH.put("device", "remote_controlboard");
        OptH.put("remote",("/"+robot+"/head"));
        OptH.put("local", ("/"+name +"/head"));

        if (!ddH.open(OptH))
        {
            yError("[vtRFThread] could not open head PolyDriver!");
            return false;
        }
        ok = 1;
        if (ddH.isValid())
        {
            ok = ok && ddH.view(iencsH);
        }
        if (!ok)
        {
            yError("[vtRFThread] Problems acquiring head interfaces!!!!");
            return false;
        }
        iencsH->getAxes(&jntsH);
        encsH = new yarp::sig::Vector(jntsH,0.0);

        yDebug("Setting up iCubSkin...");
        iCubSkinSize = filenames.size();

        for(unsigned int i=0;i<filenames.size();i++)
        {
            string filePath = filenames[i];
            yDebug("i: %i filePath: %s",i,filePath.c_str());
            skinPartPWE sP(modality);
            if ( setTaxelPosesFromFile(filePath,sP) )
            {
                iCubSkin.push_back(sP);
            }
        }
        load(); //here the representation params (bins, extent etc.) will be loaded and set to the pwe of every taxel

        yInfo("iCubSkin correctly instantiated. Size: %lu",iCubSkin.size());

        if (verbosity>= 2)
        {
            for (auto & i : iCubSkin)
            {
                i.print(verbosity);
            }
        }
        iCubSkinSize = iCubSkin.size();

    return true;
}

void vtRFThread::run()
{
    printMessage(4,"\n **************************vtRFThread::run() ************************\n");
    // read from the input ports
    dTBottle                       = dTPort       -> read(false);
    stressBottle                   = stressPort   -> read(false);
    event                          = eventsPort   -> read(false);
    imageInR                       = imagePortInR -> read(false);
    imageInL                       = imagePortInL -> read(false);
    skinContactList *skinContacts  = skinPortIn   -> read(false);

    dumpedVector.resize(0,0.0);
    readEncodersAndUpdateArmChains();
    readHeadEncodersAndUpdateEyeChains(); //has to be called after readEncodersAndUpdateArmChains(), which reads torso encoders

    if (stressBottle !=nullptr)
    {
        stress = stressBottle->get(0).asFloat64();
        yAssert((stress>=0.0) && (stress<=1.0));
        printMessage(3,"vtRFThread::run() reading %f stress value from port.\n",stress);
    }
    else
        stress = 0.0;

    // project taxels in World Reference Frame
    for (size_t i = 0; i < iCubSkin.size(); i++)
    {
        for (size_t j = 0; j < iCubSkin[i].taxels.size(); j++)
        {
            iCubSkin[i].taxels[j]->setWRFPosition(locateTaxel(iCubSkin[i].taxels[j]->getPosition(),iCubSkin[i].name));
            printMessage(7,"iCubSkin[%i].taxels[%i].WRFPos %s\n",i,j,iCubSkin[i].taxels[j]->getWRFPosition().toString().c_str());
        }
    }

    Bottle inputEvents;
    inputEvents.clear();

    if (event == nullptr)
    {
        // if there is nothing from the port but there was a previous event,
        // and it did not pass more than 0.2 seconds from the last data, let's use that
        if ((yarp::os::Time::now() - timeNow <= 0.2) && !incomingEvents.empty())
        {
            for(auto & incomingEvent : incomingEvents)
            {
                Bottle &b = inputEvents.addList();
                b = incomingEvent.toBottle();
            }
            printMessage(4,"Assigned %s to inputEvents from memory.\n",inputEvents.toString().c_str());    
        }
    }
    else
    {
        timeNow     = yarp::os::Time::now();
        inputEvents = *event; //we assign what we just read from the port 
        printMessage(4,"Read %s from events port.\n",event->toString().c_str());
    }
    //otherwise, inputEvents remains empty
    
    ts.update();
    incomingEvents.clear();
    resetTaxelEventVectors();
    
    // process the port coming from the visuoTactileWrapper
    if (inputEvents.size() != 0)
    {
        // read the events
        for (int i = 0; i < inputEvents.size(); i++)
        {
            incomingEvents.emplace_back(*(inputEvents.get(i).asList()));
            printMessage(3,"\n[EVENT] %s\n", incomingEvents.back().toString().c_str());
        }

        // manage the buffer
        if (eventsFlag)
        {
            eventsFlag  = false;
            yInfo("Starting the buffer..");
        }
        else
        {
            eventsBuffer.push_back(incomingEvents.back()); //! the buffering and hence the learning is working only for the last event in the vector
            //!so learning should be done with one stimulus only
            printMessage(2,"I'm buffering inputs (taking last event from stack)! Buffer size %lu \n",eventsBuffer.size());
        }

        // limit the size of the buffer to 80, i.e. 4 seconds of acquisition
        if (eventsBuffer.size() >= 80)
        {
            eventsBuffer.erase(eventsBuffer.begin());
            yTrace("Too many samples: removing the older element from the buffer..");
        }

        // detect contacts and train the taxels
        if (skinContacts && eventsBuffer.size()>20)
        {
            std::vector<unsigned int> IDv; IDv.clear();
            int IDx = -1;
            if (detectContact(skinContacts, IDx, IDv))
            {
                yInfo("Contact! Training the taxels..");
                timeNow     = yarp::os::Time::now();
                if (learningFlag)
                {
                    eventsFlag  = trainTaxels(IDv,IDx);
                }
                else
                {
                    yWarning("No Learning has been put in place.");
                    size_t taxelsize = iCubSkin[0].taxels.size();
                    for (size_t j = 0; j < taxelsize; j++)
                    {
                        dumpedVector.push_back(0.0);
                    }
                }
                eventsBuffer.clear();
            }
            else
            {
                size_t taxelsize = iCubSkin[0].taxels.size();
                for (size_t j = 0; j < taxelsize; j++)
                {
                    dumpedVector.push_back(0.0);
                }
            }
        }
        else
        {
            size_t taxelsize = iCubSkin[0].taxels.size();
            for (size_t j = 0; j < taxelsize; j++)
            {
                dumpedVector.push_back(0.0);
            }
        }
    }
    // if there's no input for more than 2 seconds, clear the buffer
    else if (yarp::os::Time::now() - timeNow > 2.0)
    {
        eventsFlag = true;
        eventsBuffer.clear();
        timeNow = yarp::os::Time::now();
        yInfo("No significant event in the last 2 seconds. Erasing the buffer..");

        for (size_t j = 0; j < iCubSkin[0].taxels.size(); j++)
        {
            dumpedVector.push_back(0.0);
        }
    }
    else
    {
        for (size_t j = 0; j < iCubSkin[0].taxels.size(); j++)
        {
            dumpedVector.push_back(0.0);
        }
    }

    // Superimpose the taxels onto the right eye
    if (imageInR!=nullptr)
        drawTaxels("rightEye");

    // Superimpose the taxels onto the left eye
    if (imageInL!=nullptr)
        drawTaxels("leftEye");

    if (!incomingEvents.empty())
    {
        projectIncomingEvents();     // project event onto the taxels' FoR and add them to taxels' representation
        // only if event lies inside taxel's RF 

        //should keep backwards compatibility with the dumpedVector format
        //it will currently dump only the last event of the possibly multiple events
        //TODO check whether this should be preserved or modified - output is cryptic, order of taxels needs to be known
        for (int i = 0; i < iCubSkinSize; i++)
            for (auto & taxel : iCubSkin[i].taxels)
                if(!(dynamic_cast<TaxelPWE*>(taxel))->Evnts.empty())
                {
                    //Ale: There's a reason behind this choice
                    //Matej: What does that mean?
                    dumpedVector.push_back((dynamic_cast<TaxelPWE*>(taxel))->Evnts.back().Pos[0]);
                    dumpedVector.push_back(dynamic_cast<TaxelPWE*>(taxel)->Evnts.back().Pos[1]);
                    dumpedVector.push_back(dynamic_cast<TaxelPWE*>(taxel)->Evnts.back().Pos[2]);
                }

        computeResponse(stress);    // compute the response of each taxel
    }

    sendContactsToSkinGui();       
    managePPSevents();

    // manage the dumped port
    if (dumpedVector.size()>0)
    {
        Bottle bd;
        bd.clear();

        vectorIntoBottle(dumpedVector,bd);
        dataDumperPortOut.setEnvelope(ts);
        dataDumperPortOut.write(bd);
    }
}

void vtRFThread::managePPSevents()
{
    // main/src/modules/skinManager/src/compensationThread.cpp:250
//    vector <int> taxelsIDs;
    string part = SkinPart_s[SKIN_PART_UNKNOWN];
    int iCubSkinID=-1;
//    bool isThereAnEvent = false;

    Bottle & out = ppsEventsPortOut.prepare();     out.clear();
    Bottle b;     b.clear();

    if (!incomingEvents.empty())  // if there's an event
    {
        for (int i = 0; i < iCubSkinSize; i++) // cycle through the skinparts
        {
            b.clear(); //so there will be one bottle per skin part (if there was a significant event)
            Vector geoCenter(3,0.0), normalDir(3,0.0), obsCenter(3, 0.0);
            Vector geoCenterWRF(3,0.0), normalDirWRF(3,0.0); //in world reference frame
            double w = 0.0;
            double w_max = 0.0;
            double w_sum = 0.0;
            Matrix chest_t;
            part  = iCubSkin[i].name;

            Matrix T_a = eye(4);               // transform matrix relative to the arm
            int index = -1;
            for (int j = 0; j < SKIN_PART_SIZE; j++)
            {
                if (part == SkinPart_s[j])
                    index = j;
            }
            if (SkinPart_2_BodyPart[index].body == LEFT_ARM)
            {
                T_a = armL->getH(3+SkinPart_2_LinkNum[index].linkNum, true);
            }
            else if (SkinPart_2_BodyPart[index].body == RIGHT_ARM)
            {
                T_a = armR->getH(3+SkinPart_2_LinkNum[index].linkNum, true);
            }
            else if (iCubSkin[i].name == SkinPart_s[SKIN_FRONT_TORSO]) {
                T_a = torso->getH(SkinPart_2_LinkNum[index].linkNum, true);
            }
            else
                yError("[vtRFThread] %s in projectIncomingEvent!\n", iCubSkin[i].name.c_str());


            b.addInt32(getSkinPartFromString(iCubSkin[i].name));

            for (auto & taxel : iCubSkin[i].taxels) // cycle through the taxels
            {
                //take only highly activated "taxels"
                if (dynamic_cast<TaxelPWE*>(taxel)->Resp > PPS_AGGREG_ACT_THRESHOLD)
                {
                    w = dynamic_cast<TaxelPWE*>(taxel)->Resp;
                    printMessage(0,"part %s: pps taxel ID %d, pos (%s), activation: %.2f\n",part.c_str(), taxel->getID(),taxel->getPosition().toString(3,3).c_str(),w);
                    //The final geoCenter and normalDir will be a weighted average of the activations
                    geoCenter += taxel->getPosition()*w; //Matej, 24.2., changing convention - link not Root FoR
//                    normalDir += taxel->getNormal()*w;
                    auto pos = dynamic_cast<TaxelPWE*>(taxel)->Evnts.back().Pos;
                    pos.push_back(1);
                    auto obs_pos = taxel->getFoR()*pos;
                    auto dir =  obs_pos.subVector(0,2) - taxel->getPosition();
                    dir /= norm(dir);
                    normalDir += dir*w;
                    obsCenter += dynamic_cast<TaxelPWE*>(taxel)->Evnts.back().Pos * w;
                    geoCenterWRF += taxel->getWRFPosition()*w; //original code
//                    auto dirWRF = (T_a*obs_pos).subVector(0,2) - taxel->getWRFPosition();
//                    printf("Normal is \n%s\n%s\n", (dirWRF/norm(dirWRF)).toString(3).c_str(), (T_a.submatrix(0,2,0,2)*(dir/norm(dir))).toString(3).c_str());
                    normalDirWRF += T_a.submatrix(0,2,0,2)*dir*w;
                    w_sum += w;
                    if (w>w_max)
                    {
//                        auto dir = taxel->getFoR().submatrix(0,2,0,2)*(dynamic_cast<TaxelPWE*>(taxel)->Evnts.back().Pos - taxel->getPosition());
//                        normalDir = dir/norm(dir);
//                        normalDir = taxel->getNormal();
                        w_max = w;
                    }
                }
            }
            if (w_sum > 0)
            {
                normalDir /= norm(normalDir);
                geoCenter /= w_sum;
//                normalDir /= w_sum;
                geoCenterWRF /= w_sum;
                normalDirWRF /= w_sum;
                vectorIntoBottle(geoCenter, b);
                vectorIntoBottle(normalDir, b);
                vectorIntoBottle(geoCenterWRF, b);
                vectorIntoBottle(normalDirWRF, b);
                b.addFloat64(w_max);
                //b.addDouble(w_max/255.0); // used to be this before adapting parzenWindowEstimator1D::getF_X_scaled
                //should be inside <0,1> but if the event has a >0 threat value, response is amplified and may exceed 1
                b.addString(part);
                out.addList().read(b);
            }
        }

        ppsEventsPortOut.setEnvelope(ts);
        ppsEventsPortOut.write();     // let's send only if there was en event
    }
}

void vtRFThread::sendContactsToSkinGui()
{
    Vector respToSkin;

    for(int i=0; i<iCubSkinSize; i++)
    {
        respToSkin.resize(iCubSkin[i].size,0.0);   // resize the vector to the skinPart

        if (!incomingEvents.empty())
        {
            for (size_t j = 0; j < iCubSkin[i].taxels.size(); j++)
            {
                if(iCubSkin[i].repr2TaxelList.empty())
                {
                    //we simply light up the taxels themselves
                    respToSkin[iCubSkin[i].taxels[j]->getID()] = RESP_GAIN_FOR_SKINGUI * dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Resp;
                }
                else
                {
                    //we light up all the taxels represented by the particular taxel
                    list<unsigned int> l = iCubSkin[i].repr2TaxelList[iCubSkin[i].taxels[j]->getID()];

                    if (l.empty())
                    {
                        yWarning("skinPart %d Taxel %d : no list of represented taxels is available, even if repr2TaxelList is not empty",i,iCubSkin[i].taxels[j]->getID());
                        respToSkin[iCubSkin[i].taxels[j]->getID()] = RESP_GAIN_FOR_SKINGUI * dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Resp;
                    }
                    else
                    {
                        for(unsigned int & iter_list : l)
                        {
                            //for all the represented taxels, we assign the activation of the super-taxel
                            respToSkin[iter_list] =  RESP_GAIN_FOR_SKINGUI * dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Resp;
                        }
                    }
                }
            }
        }


        Bottle colorBottle;
        colorBottle.addInt32(0);
        colorBottle.addInt32(200);
        colorBottle.addInt32(100);
        BufferedPort<Bottle> *outPort = nullptr;
        if(iCubSkin[i].name == SkinPart_s[SKIN_LEFT_FOREARM])
        {
            outPort = &skinGuiPortForearmL;
        }
        else if(iCubSkin[i].name == SkinPart_s[SKIN_RIGHT_FOREARM])
        {
            outPort = &skinGuiPortForearmR;
        }
        else if(iCubSkin[i].name == SkinPart_s[SKIN_LEFT_UPPER_ARM])
        {
            outPort = &skinGuiPortUpperarmL;
        }
        else if(iCubSkin[i].name == SkinPart_s[SKIN_RIGHT_UPPER_ARM])
        {
            outPort = &skinGuiPortUpperarmR;
        }
        else if(iCubSkin[i].name == SkinPart_s[SKIN_LEFT_HAND])
        {
            outPort = &skinGuiPortHandL;
        }
        else if(iCubSkin[i].name == SkinPart_s[SKIN_RIGHT_HAND])
        {
            outPort = &skinGuiPortHandR;
        }
        else if(iCubSkin[i].name == SkinPart_s[SKIN_FRONT_TORSO])
        {
            outPort = &skinGuiPortTorso;
        }

        Bottle dataBottle;
        dataBottle.addList().read(respToSkin);

        Bottle& outputBottle=outPort->prepare();
        outputBottle.clear();

        outputBottle.addList() = *(dataBottle.get(0).asList());
        outputBottle.addList() = colorBottle;

        outPort->setEnvelope(ts);
        outPort->write();
    }
}

bool vtRFThread::detectContact(iCub::skinDynLib::skinContactList *_sCL, int &idx,
                               std::vector <unsigned int> &idv)
{
    // Search for a suitable contact. It has this requirements:
    //   1. it has to be higher than SKIN_THRES
    //   2. more than two taxels should be active for that contact (in order to avoid spikes)
    //   3. it should be in the proper skinpart (forearms and hands)
    //   4. it should activate one of the taxels used by the module
    //      (e.g. the fingers will not be considered)
    for(auto & it : *_sCL)
    {
        idv.clear();
        if( it.getPressure() > SKIN_THRES && (it.getTaxelList()).size() > 2 )
        {
            for (int i = 0; i < iCubSkinSize; i++)
            {
                if (SkinPart_s[it.getSkinPart()] == iCubSkin[i].name)
                {
                    idx = i;
                    std::vector <unsigned int> txlList = it.getTaxelList();

                    bool itHasBeenTouched = false;

                    getRepresentativeTaxels(txlList, idx, idv);

                    if (!idv.empty())
                    {
                        itHasBeenTouched = true;
                    }

                    if (itHasBeenTouched)
                    {
                        if (verbosity>=1)
                        {
                            printMessage(1,"Contact! Skin part: %s\tTaxels' ID: ",iCubSkin[i].name.c_str());
                            for (unsigned int j : idv)
                                printf("\t%i",j);

                            printf("\n");
                        }

                        return true;
                    }
                }
            }
        }
    }
    return false;
}

string vtRFThread::load()
{
    string fileName=rf->findFile("taxelsFile");
    if (fileName=="")
    {
        yWarning("[vtRF::load] No filename has been found. Skipping..");
        string ret;
        return ret;
    }

    yInfo("[vtRF::load] File loaded: %s", fileName.c_str());
    Property data; data.fromConfigFile(fileName);
    Bottle b; b.read(data);
    yDebug("[vtRF::load] iCubSkinSize %i",iCubSkinSize);

    for (int i = 0; i < iCubSkinSize; i++)
    {
        Bottle bb = b.findGroup(iCubSkin[i].name);

        if (bb.size() > 0)
        {
            string modal    = bb.find("modality").asString();
            int nTaxels     = bb.find("nTaxels").asInt32();
            int size        = bb.find("size").asInt32();

            iCubSkin[i].size     = size;
            iCubSkin[i].modality = modal;

            Matrix ext;
            std::vector<int>    bNum;
            std::vector<int>    mapp;

            Bottle *bbb;
            bbb = bb.find("ext").asList();
            if (modal=="1D")
            {
                ext = matrixFromBottle(*bbb,0,1,2); //e.g.  ext  (-0.1 0.2) ~ (min max) will become [-0.1 0.2]
            }
            else
            {
                ext = matrixFromBottle(*bbb,0,2,2); //e.g. (-0.1 0.2 0.0 1.2) ~ (min_distance max_distance min_TTC max_TTC) 
                //will become [min_distance max_distance ; min_TTC max_TTC] 
            }

            bbb = bb.find("binsNum").asList();
            bNum.push_back(bbb->get(0).asInt32());
            bNum.push_back(bbb->get(1).asInt32());

            bbb = bb.find("Mapping").asList();
            yDebug("[vtRF::load][%s] size %i\tnTaxels %i\text %s\tbinsNum %i %i",iCubSkin[i].name.c_str(),size,
                                                  nTaxels,toVector(ext).toString(3,3).c_str(),bNum[0],bNum[1]);
            printMessage(3,"Mapping\n");
            for (int j = 0; j < size; j++)
            {
                mapp.push_back(bbb->get(j).asInt32());
                if (verbosity>=3)
                {
                    printf("%i ",mapp[j]);
                }
            }
            if (verbosity>=3) printf("\n");
            iCubSkin[i].taxel2Repr = mapp;

            for (int j = 0; j < nTaxels; j++)
            {
                // 7 are the number of lines in the skinpart group that are not taxels
                bbb = bb.get(j+7).asList();
                printMessage(3,"Reading taxel %s\n",bbb->toString().c_str());

                for (auto & taxel : iCubSkin[i].taxels)
                {
                    if (taxel->getID() == bbb->get(0).asInt32())
                    {
                        if (dynamic_cast<TaxelPWE*>(taxel)->pwe->resize(ext,bNum))
                        {
                            dynamic_cast<TaxelPWE*>(taxel)->pwe->setPosHist(matrixFromBottle(*bbb->get(1).asList(),0,bNum[0],bNum[1]));
                            dynamic_cast<TaxelPWE*>(taxel)->pwe->setNegHist(matrixFromBottle(*bbb->get(2).asList(),0,bNum[0],bNum[1]));
                        }
                    }
                }
            }
        }
    }

    return fileName;
}

string vtRFThread::save()
{
    int    lastindex     = taxelsFile.find_last_of('.');
    string taxelsFileRaw = taxelsFile.substr(0, lastindex);

    string fnm=path+taxelsFileRaw+"_out.ini";
    ofstream myfile;
    yInfo("Saving to: %s", fnm.c_str());
    myfile.open(fnm.c_str(),ios::trunc);

    if (myfile.is_open())
    {
        for (int i = 0; i < iCubSkinSize; i++)
        {
            Bottle data;
            data.clear();

            Matrix           getExt = dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[0])->pwe->getExt();
            matrixIntoBottle(getExt,data);

            std::vector<int> bNum  = dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[0])->pwe->getHistSize();

            myfile << "[" << iCubSkin[i].name << "]" << endl;
            myfile << "modality\t"  << iCubSkin[i].modality << endl;
            myfile << "size    \t"  << iCubSkin[i].size << endl;
            myfile << "nTaxels \t"  << iCubSkin[i].taxels.size() << endl;
            myfile << "ext     \t(" << data.toString() << ")\n";
            myfile << "binsNum \t(" << bNum[0] << "\t" << bNum[1] << ")\n";

            data.clear();
            Bottle &representatives = data.addList();
            for (int q : iCubSkin[i].taxel2Repr)
            {
                representatives.addInt32(q);
            }
            myfile << "Mapping\t" << data.toString() << endl;

            for (auto & taxel : iCubSkin[i].taxels)
            {
                data.clear();
                data=dynamic_cast<TaxelPWE*>(taxel)->TaxelPWEIntoBottle();
                myfile << data.toString() << "\n";
            }
        }
    }
    myfile.close();
    return fnm;
}

bool vtRFThread::trainTaxels(const std::vector<unsigned int>& IDv, const int IDx)
{
    std::vector<unsigned int> v;
    getRepresentativeTaxels(IDv, IDx, v);

    Matrix T_a = eye(4);                     // transform matrix relative to the arm
    int index = -1;
    for (int j = 0; j < SKIN_PART_SIZE; j++)
    {
        if (iCubSkin[IDx].name == SkinPart_s[j])
            index = j;
    }
    if (SkinPart_2_BodyPart[index].body == LEFT_ARM)
    {
        T_a = armL->getH(3+SkinPart_2_LinkNum[index].linkNum, true);
    }
    else if (SkinPart_2_BodyPart[index].body == RIGHT_ARM)
    {
        T_a = armR->getH(3+SkinPart_2_LinkNum[index].linkNum, true);
    }
    else if (iCubSkin[IDx].name == SkinPart_s[SKIN_FRONT_TORSO])
    {
        T_a = torso->getH(2, true);
    }
    else
    {
        yError("[vtRFThread] in trainTaxels!\n");
        return false;
    }

    for (size_t j = 0; j < iCubSkin[IDx].taxels.size(); j++)
    {
        bool itHasBeenTouched = false;
        for (unsigned int w : v)
        {
            if (iCubSkin[IDx].taxels[j]->getID() == w)
            {
                itHasBeenTouched = true;
            }
        }

        for (size_t k = 0; k < eventsBuffer.size(); k++)
        {
            IncomingEvent4TaxelPWE projection = projectIntoTaxelRF(iCubSkin[IDx].taxels[j]->getFoR(),T_a,eventsBuffer[k]); //project's into taxel RF and subtracts object radius from z pos in the new frame
            if(dynamic_cast<TaxelPWE*>(iCubSkin[IDx].taxels[j])->insideRFCheck(projection)){ //events outside of taxel's RF will not be used for learning
                if (itHasBeenTouched){ 
                    printMessage(2,"Training taxel - positive sample: skinPart %d ID %i k (index in buffer) %i NORM %g TTC %g\n",IDx,iCubSkin[IDx].taxels[j]->getID(),k,projection.NRM,projection.TTC);
                    dynamic_cast<TaxelPWE*>(iCubSkin[IDx].taxels[j])->addSample(projection);
                    dumpedVector.push_back(1.0);
                }
                else{
                    printMessage(2,"Training taxel - negative sample: skinPart %d ID %i k (index in buffer) %i NORM %g TTC %g\n",IDx,iCubSkin[IDx].taxels[j]->getID(),k,projection.NRM,projection.TTC);
                    dynamic_cast<TaxelPWE*>(iCubSkin[IDx].taxels[j])->removeSample(projection);  
                    dumpedVector.push_back(-1.0);
                }
            }
            else
                printMessage(3,"Not training taxel: event outside RF\n");
        }
    }

    return true;
}

bool vtRFThread::readEncodersAndUpdateArmChains()
{    
   Vector q1(jntsT+jntsAR,0.0);
   Vector q2(jntsT+jntsAL,0.0);
       
   iencsT->getEncoders(encsT->data());
   qT[0]=(*encsT)[2]; //reshuffling from motor to iKin order (yaw, roll, pitch)
   qT[1]=(*encsT)[1];
   qT[2]=(*encsT)[0];

   if (rf->check("rightHand") || rf->check("rightForeArm") ||
        (!rf->check("rightHand") && !rf->check("rightForeArm") && !rf->check("leftHand") && !rf->check("leftForeArm")))
   {
        iencsR->getEncoders(encsR->data());
        qR=encsR->subVector(0,jntsAR-1);
        q1.setSubvector(0,qT);
        q1.setSubvector(jntsT,qR);
        armR -> setAng(q1*CTRL_DEG2RAD);
   }
   if (rf->check("leftHand") || rf->check("leftForeArm") ||
           (!rf->check("rightHand") && !rf->check("rightForeArm") && !rf->check("leftHand") && !rf->check("leftForeArm")))
   {
        iencsL->getEncoders(encsL->data());
        qL=encsL->subVector(0,jntsAL-1);
        q2.setSubvector(0,qT);
        q2.setSubvector(jntsT,qL);
        armL -> setAng(q2*CTRL_DEG2RAD);
   }
   torso->setAng(qT*CTRL_DEG2RAD);
     
   return true;
}

bool vtRFThread::readHeadEncodersAndUpdateEyeChains()
{
    iencsH->getEncoders(encsH->data());
    yarp::sig::Vector  head=*encsH;

    yarp::sig::Vector q(8);
    q[0]=qT[0];       q[1]=qT[1];        q[2]=qT[2];
    q[3]=head[0];        q[4]=head[1];
    q[5]=head[2];        q[6]=head[3];
   
    //left eye
    q[7]=head[4]+head[5]/2.0;
    eWL->eye->setAng(q*CTRL_DEG2RAD);  
    //right eye
    q[7]=head[4]-head[5]/2.0;
    eWR->eye->setAng(q*CTRL_DEG2RAD);
    
    return true;
}

bool vtRFThread::projectIncomingEvents()
{
    for (const auto & incomingEvent : incomingEvents)
    {
        for (int i = 0; i < iCubSkinSize; i++)
        {
            Matrix T_a = eye(4);               // transform matrix relative to the arm
            int index = -1;
            for (int j = 0; j < SKIN_PART_SIZE; j++)
            {
                if (iCubSkin[i].name == SkinPart_s[j])
                    index = j;
            }
            if (SkinPart_2_BodyPart[index].body == LEFT_ARM)
            {
                T_a = armL->getH(3+SkinPart_2_LinkNum[index].linkNum, true);
            }
            else if (SkinPart_2_BodyPart[index].body == RIGHT_ARM)
            {
                T_a = armR->getH(3+SkinPart_2_LinkNum[index].linkNum, true);
            }
            else if (iCubSkin[i].name == SkinPart_s[SKIN_FRONT_TORSO]) {
                T_a = torso->getH(SkinPart_2_LinkNum[index].linkNum, true);
            }
            else
                yError("[vtRFThread] %s in projectIncomingEvent!\n", iCubSkin[i].name.c_str());

//            yInfo("T_A:\n%s",T_a.toString().c_str());
            printMessage(5,"\nProject incoming event %s \t onto %s taxels\n",incomingEvent.toString().c_str(),iCubSkin[i].name.c_str());
            IncomingEvent4TaxelPWE projEvent; 
            for (auto & taxel : iCubSkin[i].taxels)
            {
                projEvent = projectIntoTaxelRF(taxel->getFoR(),T_a,incomingEvent); //project's into taxel RF and subtracts object radius from z pos in the new frame
                if(dynamic_cast<TaxelPWE*>(taxel)->insideRFCheck(projEvent)) ////events outside of taxel's RF will not be added
                {
                    dynamic_cast<TaxelPWE*>(taxel)->Evnts.push_back(projEvent); //here every taxel (TaxelPWE) is updated with the events
                    printMessage(6,"    Projecting onto taxel %d (Pos in Root FoR: %s Pos in local FoR: %s).\n",taxel->getID(),
                                 taxel->getWRFPosition().toString().c_str(),
                                 taxel->getPosition().toString().c_str());
                    printMessage(6,"\tProjected event: %s\n",projEvent.toString().c_str());
                    printMessage(6,"\tIncoming event: %s\n",incomingEvent.toString().c_str());
                    printMessage(6,"\tLies inside RF - pushing to taxelPWE.Events.\n");
                }
                else
                    printMessage(6,"\tLies outside RF.\n");

                //printMessage(5,"Repr. taxel ID %i\tEvent: %s\n",j,dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Evnt.toString().c_str());
            }
        }
    }
    return true;
}

IncomingEvent4TaxelPWE vtRFThread::projectIntoTaxelRF(const Matrix &RF,const Matrix &T_a,const IncomingEvent &e)
{
    IncomingEvent4TaxelPWE Event_projected = e;

    Matrix T_a_proj = T_a * RF;

    Vector p=e.Pos; p.push_back(1);
    Vector v=e.Vel; v.push_back(1);

    Event_projected.Pos = SE3inv(T_a_proj)*p;        Event_projected.Pos.pop_back();
    Event_projected.Vel = SE3inv(T_a_proj)*v;        Event_projected.Vel.pop_back();

    if (e.Radius != -1.0)
    {
        Event_projected.Pos(2) -= Event_projected.Radius; //considering the radius, this brings the object closer in z  by the radius
        //for the rest of the calculations (in particular in x,y), the object is treated as a point
    }

    Event_projected.computeNRMTTC();

    return Event_projected;
}

void vtRFThread::resetTaxelEventVectors()
{
    printMessage(4,"[vtRFThread::resetTaxelEventVectors()]\n");
    for (int i = 0; i < iCubSkinSize; i++)
        for (auto & taxel : iCubSkin[i].taxels)
            (dynamic_cast<TaxelPWE*>(taxel))->Evnts.clear();        
}

void vtRFThread::resetParzenWindows()
{
    for (int i = 0; i < iCubSkinSize; i++)
    {
        for (auto & taxel : iCubSkin[i].taxels)
        {
            dynamic_cast<TaxelPWE*>(taxel)->resetParzenWindowEstimator();
        }
    }
}

bool vtRFThread::computeResponse(double stress_modulation)
{
    printMessage(4,"\n\n *** [vtRFThread::computeResponse] Taxel responses ***:\n");
    for (int i = 0; i < iCubSkinSize; i++)
    {
        printMessage(4,"\n ** %s ** \n",iCubSkin[i].name.c_str());
        for (size_t j = 0; j < iCubSkin[i].taxels.size(); j++)
        {
            dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->computeResponse(stress_modulation);
            printMessage(4,"\t %ith (ID: %d) %s taxel response %.2f (with stress modulation:%.2f)\n",j,iCubSkin[i].taxels[j]->getID(),iCubSkin[i].name.c_str(),dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Resp,stress_modulation);
        }
    }

    return true;
}

bool vtRFThread::stopLearning()
{
    learningFlag = false;
    return true;
}

bool vtRFThread::restoreLearning()
{
    learningFlag = true;
    return true;
}

bool vtRFThread::pushExtrinsics(const Matrix &M, const string& eye)
{
    if (eye == "leftEye")
    {
        eWL->eye->setHN(M);
    }
    else if (eye == "rightEye")
    {
        eWR->eye->setHN(M);
    }
    else
        return false;

    return true;
}

void vtRFThread::drawTaxels(const string& _eye)
{
    projectIntoImagePlane(iCubSkin,_eye);

    ImageOf<PixelRgb> imgOut;

    if (_eye=="rightEye")
    {
        imgOut.copy(*imageInR);
    }
    else if (_eye=="leftEye")
    {
        imgOut.copy(*imageInL);
    }
    else
    {
        yError("[vtRFThread] Error in drawTaxels! Returning..");
        return;
    }

    for (int i = 0; i < iCubSkinSize; i++)
    {
        for (size_t j = 0; j < iCubSkin[i].taxels.size(); j++)
        {
            drawTaxel(imgOut,iCubSkin[i].taxels[j]->getPx(),iCubSkin[i].name,dynamic_cast<TaxelPWE*>(iCubSkin[i].taxels[j])->Resp);
            printMessage(6,"iCubSkin[%i].taxels[%i].px %s\n",i,j,iCubSkin[i].taxels[j]->getPx().toString().c_str());
        }
    }

    _eye=="rightEye"?imagePortOutR.setEnvelope(ts):imagePortOutL.setEnvelope(ts);
    _eye=="rightEye"?imagePortOutR.write(imgOut):imagePortOutL.write(imgOut);
}

void vtRFThread::drawTaxel(ImageOf<PixelRgb> &Im, const yarp::sig::Vector &px,
                           const string &part, const int act)
{
    int u = (int)px(0);
    int v = (int)px(1);
    int r = RADIUS;

    if (act>0)
        r+=1;

    if ((u >= r) && (u <= 320 - r) && (v >= r) && (v <= 240 - r))
    {
        for (int x=0; x<2*r; x++)
        {
            for (int y=0; y<2*r; y++)
            {
                if (part == SkinPart_s[SKIN_LEFT_FOREARM] || part == SkinPart_s[SKIN_RIGHT_FOREARM] ||
                    part == SkinPart_s[SKIN_LEFT_HAND]    || part == SkinPart_s[SKIN_RIGHT_HAND])
                {
                    if (act>0)
                    {
                        (Im.pixel(u+x-r,v+y-r)).r = 50;
                        (Im.pixel(u+x-r,v+y-r)).g = 50+act*205/255;
                        (Im.pixel(u+x-r,v+y-r)).b = 100-act*200/255;
                    }
                    else
                    {
                        (Im.pixel(u+x-r,v+y-r)).r = 50;
                        (Im.pixel(u+x-r,v+y-r)).g = 50;
                        (Im.pixel(u+x-r,v+y-r)).b = 100;
                    }
                }
            }
        }
    }
}

bool vtRFThread::projectIntoImagePlane(vector <skinPartPWE> &sP, const string &eye)
{
    for (auto & i : sP)
    {
        for (auto & taxel : i.taxels)
        {
            if (eye=="rightEye" || eye=="leftEye")
            {
                yarp::sig::Vector WRFpos = taxel->getWRFPosition();
                yarp::sig::Vector px     = taxel->getPx();
                projectPoint(WRFpos,px,eye);
                taxel->setPx(px);
            }
            else
            {
                yError("[vtRFThread] ERROR in projectIntoImagePlane!\n");
                return false;
            }
        }
    }

    return true;
}

bool vtRFThread::projectPoint(const yarp::sig::Vector &x,
                              yarp::sig::Vector &px, const string &_eye)
{
    if (x.length()<3)
    {
        fprintf(stdout,"Not enough values given for the point!\n");
        return false;
    }

    bool isLeft=(_eye=="leftEye");

    yarp::sig::Matrix  *Prj=(isLeft?eWL->Prj:eWR->Prj);
    iCubEye            *eye=(isLeft?eWL->eye:eWR->eye);

    if (Prj)
    {
        yarp::sig::Vector xo=x;
        if (xo.length()<4)
            xo.push_back(1.0);  // impose homogeneous coordinates
        yarp::sig::Vector xe;
        // find position wrt the camera frame
        xe=SE3inv(eye->getH())*xo;

        // find the 2D projection
        px=*Prj*xe;
        px=px/px[2];
        px.pop_back();

        return true;
    }
    else
    {
        fprintf(stdout,"Unspecified projection matrix for %s camera!\n",_eye.c_str());
        return false;
    }
}

yarp::sig::Vector vtRFThread::locateTaxel(const yarp::sig::Vector &_pos, const string &part)
{
    yarp::sig::Vector pos=_pos;
    yarp::sig::Vector WRFpos(4,0.0);
    Matrix T = eye(4);

    //printMessage(7,"locateTaxel(): Pos local frame: %s, skin part name: %s\n",_pos.toString(3,3).c_str(),part.c_str());
//    if (!((part == SkinPart_s[SKIN_LEFT_FOREARM]) || (part == SkinPart_s[SKIN_LEFT_HAND]) ||
//         (part == SkinPart_s[SKIN_RIGHT_FOREARM]) || (part == SkinPart_s[SKIN_RIGHT_HAND])))
//        yError("[vtRFThread] locateTaxel() failed - unknown skinPart!\n");

    int index = -1;
    for (int j = 0; j < SKIN_PART_SIZE; j++)
    {
        if (part == SkinPart_s[j])
            index = j;
    }
    if (SkinPart_2_BodyPart[index].body == LEFT_ARM)
    {
        T = armL->getH(3+SkinPart_2_LinkNum[index].linkNum, true);
    }
    else if (SkinPart_2_BodyPart[index].body == RIGHT_ARM)
    {
        T = armR->getH(3+SkinPart_2_LinkNum[index].linkNum, true);
    }
    else if (part == SkinPart_s[SKIN_FRONT_TORSO])
    {
        T = torso->getH(2, true);
    }
    else
        yError("[vtRFThread] locateTaxel() failed!\n");

    //printMessage(8,"    T Matrix: \n %s \n ",T.toString(3,3).c_str());
    pos.push_back(1);
    WRFpos = T * pos;
    WRFpos.pop_back();

    return WRFpos;
}

//see also Compensator::setTaxelPosesFromFile in icub-main/src/modules/skinManager/src/compensator.cpp
bool vtRFThread::setTaxelPosesFromFile(const string filePath, skinPartPWE &sP)
{
    string line;
    ifstream posFile;
    yarp::sig::Vector taxelPos(3,0.0);
    yarp::sig::Vector taxelNrm(3,0.0);
    yarp::sig::Vector taxelPosNrm(6,0.0);

    string filename = strrchr(filePath.c_str(), '/');
    filename = filename.c_str() ? filename.c_str() + 1 : filePath.c_str();

    // Assign the name and version of the skinPart according to the filename (hardcoded)
    if      (filename == "left_forearm_mesh.txt")    { sP.setName(SkinPart_s[SKIN_LEFT_FOREARM]);    sP.setVersion("V1");   }
    else if (filename == "left_forearm_nomesh.txt")  { sP.setName(SkinPart_s[SKIN_LEFT_FOREARM]);    sP.setVersion("V1");   }
    else if (filename == "left_forearm_V2.txt")      { sP.setName(SkinPart_s[SKIN_LEFT_FOREARM]);    sP.setVersion("V2");   }
    else if (filename == "left_arm_mesh.txt")        { sP.setName(SkinPart_s[SKIN_LEFT_UPPER_ARM]);  sP.setVersion("V1");   }
    else if (filename == "right_arm_mesh.txt")       { sP.setName(SkinPart_s[SKIN_RIGHT_UPPER_ARM]); sP.setVersion("V1");   }
    else if (filename == "torso.txt")                { sP.setName(SkinPart_s[SKIN_FRONT_TORSO]);     sP.setVersion("V1");   }
    else if (filename == "right_forearm_mesh.txt")   { sP.setName(SkinPart_s[SKIN_RIGHT_FOREARM]);   sP.setVersion("V1");   }
    else if (filename == "right_forearm_nomesh.txt") { sP.setName(SkinPart_s[SKIN_RIGHT_FOREARM]);   sP.setVersion("V1");   }
    else if (filename == "right_forearm_V2.txt")     { sP.setName(SkinPart_s[SKIN_RIGHT_FOREARM]);   sP.setVersion("V2");   }
    else if (filename == "left_hand_V2_1.txt")       { sP.setName(SkinPart_s[SKIN_LEFT_HAND]);       sP.setVersion("V2.1"); }
    else if (filename == "right_hand_V2_1.txt")      { sP.setName(SkinPart_s[SKIN_RIGHT_HAND]);      sP.setVersion("V2.1"); }
    else if (filename == "left_hand_Full.txt")       { sP.setName(SkinPart_s[SKIN_LEFT_HAND]);       sP.setVersion("V2.1"); }
    else if (filename == "right_hand_Full.txt")      { sP.setName(SkinPart_s[SKIN_RIGHT_HAND]);      sP.setVersion("V2.1"); }
    else
    {
        yError("[vtRFThread] Unexpected skin part file name: %s.\n",filename.c_str());
        return false;
    }
    //filename = filename.substr(0, filename.find_last_of("_"));

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("skinGui");            //overridden by --context parameter
    rf.setDefaultConfigFile(filePath); //overridden by --from parameter
    if (!rf.configure(0,nullptr))
    {
        yError("[vtRFThread] ResourceFinder was not configured correctly! Filename:");
        yError("%s",filename.c_str());
        return false;
    }

    yarp::os::Bottle &calibration = rf.findGroup("calibration");
    if (calibration.isNull())
    {
        yError("[vtRFThread::setTaxelPosesFromFile] No calibration group found!");
        return false;
    }
    printMessage(6,"[vtRFThread::setTaxelPosesFromFile] found %i taxels (not all of them are valid taxels).\n", calibration.size()-1);

    sP.spatial_sampling = "triangle";
    // First item of the bottle is "calibration", so we should not use it
    int j = 0; //this will be for the taxel IDs
    for (int i = 1; i < calibration.size(); i++)
    {
        taxelPosNrm = vectorFromBottle(*(calibration.get(i).asList()),0,6);
        taxelPos = taxelPosNrm.subVector(0,2);
        taxelNrm = taxelPosNrm.subVector(3,5);
        j = i-1;   //! note that i == line in the calibration group of .txt file;  taxel_ID (j) == line nr (i) - 1 
        printMessage(10,"[vtRFThread::setTaxelPosesFromFile] reading %i th row: taxelPos: %s\n", i,taxelPos.toString(3,3).c_str());
         
        if (sP.name == SkinPart_s[SKIN_LEFT_FOREARM] || sP.name == SkinPart_s[SKIN_RIGHT_FOREARM])
        {
            // the taxels at the centers of respective triangles  -
            // e.g. first triangle of forearm arm is at lines 1-12, center at line 4, taxelID = 3           
            if(  (j==3) || (j==15)  ||  (j==27) ||  (j==39) ||  (j==51) ||  (j==63) ||  (j==75) ||  (j==87) ||
                (j==99) || (j==111) || (j==123) || (j==135) || (j==147) || (j==159) || (j==171) || (j==183) || //end of full lower patch
                ((j==195) && (sP.version=="V2")) || (j==207) || ((j==231) && (sP.version=="V2")) || ((j==255) && (sP.version=="V1")) ||
                ((j==267) && (sP.version=="V2")) || (j==291) || (j==303) || ((j==315) && (sP.version=="V1")) || (j==339) || (j==351) )

            // if(  (j==3) ||  (j==39) || (j==207) || (j==255) || (j==291)) // Taxels that are evenly distributed throughout the forearm
                                                                         // in order to cover it as much as we can
            // if(  (j==3) ||  (j==15) ||  (j==27) || (j==183)) // taxels that are in the big patch but closest to the little patch (internally)
                                                                // 27 is proximal, 15 next, 3 next, 183 most distal
            // if((j==135) || (j==147) || (j==159) || (j==171))  // this is the second column, farther away from the stitch
                                                                 // 159 is most proximal, 147 is next, 135 next,  171 most distal
            // if((j==87) || (j==75)  || (j==39)|| (j==51)) // taxels that are in the big patch and closest to the little patch (externally)
            //                                              // 87 most proximal, 75 then, 39 then, 51 distal

            // if((j==27)  || (j==15)  || (j==3)   || (j==183) ||              // taxels used for the experimentations on the PLOS paper
            //    (j==135) || (j==147) || (j==159) || (j==171) ||
            //    (j==87)  || (j==75)  || (j==39)  || (j==51))

            // if((j==27)  || (j==15)  || (j==3)   || (j==183) ||              // taxels used for the experimentations on the IROS paper
            //    (j==87)  || (j==75)  || (j==39)  || (j==51))
            {
                sP.size++;
                if (modality=="1D")
                {
                    sP.taxels.push_back(new TaxelPWE1D(taxelPos,taxelNrm,j));
                }
                else
                {
                    sP.taxels.push_back(new TaxelPWE2D(taxelPos,taxelNrm,j));
                }
            }
            else
            {
                sP.size++;
            }
        }
        else if (sP.name == SkinPart_s[SKIN_LEFT_UPPER_ARM])
        {
            if((j==39) || (j==51) || (j==63) || (j==75) || (j==87) || (j==111) || (j==123) || (j==135) || (j==195) || (j==207) ||
                (j==267) || (j==291) || (j==303) || (j==315) || (j==327) || (j==339) || (j==351) || (j==363) || (j==375) || (j==387) ||
                (j==399) || (j==411) || (j==423) || (j==435) || (j==447) || (j==459) || (j==471) || (j==483) || (j==531) || (j==543) ||
                (j==579) || (j==639) || (j==675) || (j==687) || (j==699) || (j==711) || (j==735) || (j==759))
            {
                sP.size++;
                if (modality=="1D")
                {
                    sP.taxels.push_back(new TaxelPWE1D(taxelPos,taxelNrm,j));
                }
                else
                {
                    sP.taxels.push_back(new TaxelPWE2D(taxelPos,taxelNrm,j));
                }
            }
            else
            {
                sP.size++;
            }
        }
        else if (sP.name == SkinPart_s[SKIN_RIGHT_UPPER_ARM])
        {
            if((j==39) || (j==51) || (j==63) || (j==75) || (j==87) || (j==111) || (j==123) || (j==135) || (j==195) || (j==207) ||
                (j==219) || (j==231) || (j==243) || (j==255) || (j==267) || (j==279) || (j==291) || (j==339) || (j==351) || (j==387) ||
                (j==399) || (j==459) || (j==483) || (j==495) || (j==507) || (j==519) || (j==531) || (j==543) || (j==555) || (j==567) ||
                (j==579) || (j==639) || (j==675) || (j==687) || (j==699) || (j==711) || (j==735) || (j==759))
            {
                sP.size++;
                if (modality=="1D")
                {
                    sP.taxels.push_back(new TaxelPWE1D(taxelPos,taxelNrm,j));
                }
                else
                {
                    sP.taxels.push_back(new TaxelPWE2D(taxelPos,taxelNrm,j));
                }
            }
            else
            {
                sP.size++;
            }
        }
        else if (sP.name == SkinPart_s[SKIN_LEFT_HAND])
        { //we want to represent the 48 taxels of the palm (ignoring fingertips) with 5 taxels -
         // manually marking 5 regions of the palm and selecting their "centroids" as the representatives
            if((j==99) || (j==101) || (j==109) || (j==122) || (j==134))
            {
                sP.size++;
                if (modality=="1D")
                {
                    sP.taxels.push_back(new TaxelPWE1D(taxelPos,taxelNrm,j));
                }
                else
                {
                    sP.taxels.push_back(new TaxelPWE2D(taxelPos,taxelNrm,j));
                }
            }
            else
            {
                sP.size++;
            }
        }
        else if (sP.name == SkinPart_s[SKIN_RIGHT_HAND])
        { //right hand has different taxel nr.s than left hand
            if((j==101) || (j==103) || (j==118) || (j==137) || (j==124))
            {
                sP.size++;
                if (modality=="1D")
                {
                    sP.taxels.push_back(new TaxelPWE1D(taxelPos,taxelNrm,j));
                }
                else
                {
                    sP.taxels.push_back(new TaxelPWE2D(taxelPos,taxelNrm,j));
                }
            }
            else
            {
                sP.size++;
            }
        }
        else if (sP.name == SkinPart_s[SKIN_FRONT_TORSO])
        {
            if((j==63) || (j==75) || (j==87) || (j==111) || (j==123) || (j==135) || (j==207) || (j==219) || (j==231) || (j==267) ||
               (j==279) || (j==291) || (j==303) || (j==315) || (j==327) || (j==339) || (j==351) || (j==387) || (j==399) || (j==411) ||
               (j==423) || (j==435) || (j==447) || (j==459) || (j==471) || (j==483) || (j==495) || (j==507) || (j==519) || (j==531) ||
               (j==543) || (j==555) || (j==567) || (j==591) || (j==603) || (j==627) || (j==675) || (j==687) || (j==699) || (j==711) ||
               (j==723) || (j==735) || (j==747))
            {
                sP.size++;
                if (modality=="1D")
                {
                    sP.taxels.push_back(new TaxelPWE1D(taxelPos,taxelNrm,j));
                }
                else
                {
                    sP.taxels.push_back(new TaxelPWE2D(taxelPos,taxelNrm,j));
                }
            }
            else
            {
                sP.size++;
            }
        }
    }
    if (sP.name == SkinPart_s[SKIN_LEFT_HAND])
    {
        sP.taxels.push_back(new TaxelPWE1D({0.017280, -0.084450, 0.0},{0.0,0.0,-1.0},200));
        sP.taxels.push_back(new TaxelPWE1D({0.073922, -0.060010, 0.0},{0.0,0.0,-1.0},201));
        sP.taxels.push_back(new TaxelPWE1D({0.087100, -0.008302, 0.0},{0.0,0.0,-1.0},202));
        sP.taxels.push_back(new TaxelPWE1D({0.075790, 0.035530, 0.0},{0.0,0.0,-1.0},203));
        sP.taxels.push_back(new TaxelPWE1D({0.067265, 0.049148, 0.0},{0.0,0.0,-1.0},204));
        sP.taxels.push_back(new TaxelPWE1D({0.017280, -0.084450, 0.0},{0.0,0.0,1.0},205));
        sP.taxels.push_back(new TaxelPWE1D({0.073922, -0.060010, 0.0},{0.0,0.0,1.0},206));
        sP.taxels.push_back(new TaxelPWE1D({0.087100, -0.008302, 0.0},{0.0,0.0,1.0},207));
        sP.taxels.push_back(new TaxelPWE1D({0.075790, 0.035530, 0.0},{0.0,0.0,1.0},208));
        sP.taxels.push_back(new TaxelPWE1D({0.067265, 0.049148, 0.0},{0.0,0.0,1.0},209));
        sP.taxels.push_back(new TaxelPWE1D({-0.0014, 0.0212, 0.0},{0.0,0.0,1.0},210));
        sP.taxels.push_back(new TaxelPWE1D({0.0001, 0.0012, 0.0},{0.0,0.0,1.0},211));
        sP.taxels.push_back(new TaxelPWE1D({-0.0164, 0.0162, 0.0},{0.0,0.0,1.0},212));
        sP.taxels.push_back(new TaxelPWE1D({0.0001, -0.0193, 0.0},{0.0,0.0,1.0},213));
        sP.taxels.push_back(new TaxelPWE1D({-0.0254, 0.0122, 0.0},{0.0,0.0,1.0},214));
        sP.taxels.push_back(new TaxelPWE1D({0.07, 0.052, 0.0},{0.0,1.0,0.0},215));
        sP.taxels.push_back(new TaxelPWE1D({0.04, 0.052, 0.0},{0.0,1.0,0.0},216));
        sP.taxels.push_back(new TaxelPWE1D({0.01, 0.052, 0.0},{0.0,1.0,0.0},217));
        sP.taxels.push_back(new TaxelPWE1D({-0.03, 0.052, 0.0},{0.0,1.0,0.0},218));
    }
    if (sP.name == SkinPart_s[SKIN_RIGHT_HAND])
    {
        sP.taxels.push_back(new TaxelPWE1D({0.017280, -0.084450, 0.0},{0.0,0.0,1.0},200));
        sP.taxels.push_back(new TaxelPWE1D({0.073922, -0.060010, 0.0},{0.0,0.0,1.0},201));
        sP.taxels.push_back(new TaxelPWE1D({0.087100, -0.008302, 0.0},{0.0,0.0,1.0},202));
        sP.taxels.push_back(new TaxelPWE1D({0.075790, 0.035530, 0.0},{0.0,0.0,1.0},203));
        sP.taxels.push_back(new TaxelPWE1D({0.067265, 0.049148, 0.0},{0.0,0.0,1.0},204));
        sP.taxels.push_back(new TaxelPWE1D({0.017280, -0.084450, -0.02},{0.0,0.0,-1.0},205));
        sP.taxels.push_back(new TaxelPWE1D({0.073922, -0.060010, -0.02},{0.0,0.0,-1.0},206));
        sP.taxels.push_back(new TaxelPWE1D({0.087100, -0.008302, -0.02},{0.0,0.0,-1.0},207));
        sP.taxels.push_back(new TaxelPWE1D({0.075790, 0.035530, -0.02},{0.0,0.0,-1.0},208));
        sP.taxels.push_back(new TaxelPWE1D({0.067265, 0.049148, -0.02},{0.0,0.0,-1.0},209));
        sP.taxels.push_back(new TaxelPWE1D({-0.0014, 0.0212, -0.02},{0.0,0.0,-1.0},210));
        sP.taxels.push_back(new TaxelPWE1D({0.0001, 0.0012, -0.02},{0.0,0.0,-1.0},211));
        sP.taxels.push_back(new TaxelPWE1D({-0.0164, 0.0162, -0.02},{0.0,0.0,-1.0},212));
        sP.taxels.push_back(new TaxelPWE1D({0.0001, -0.0193, -0.02},{0.0,0.0,-1.0},213));
        sP.taxels.push_back(new TaxelPWE1D({-0.0254, 0.0122, -0.02},{0.0,0.0,-1.0},214));
        sP.taxels.push_back(new TaxelPWE1D({0.07, 0.052, 0.0},{0.0,1.0,0.0},215));
        sP.taxels.push_back(new TaxelPWE1D({0.04, 0.052, 0.0},{0.0,1.0,0.0},216));
        sP.taxels.push_back(new TaxelPWE1D({0.01, 0.052, 0.0},{0.0,1.0,0.0},217));
        sP.taxels.push_back(new TaxelPWE1D({-0.03, 0.052, 0.0},{0.0,1.0,0.0},218));
    }

    initRepresentativeTaxels(sP);

    return true;
}

void vtRFThread::initRepresentativeTaxels(skinPart &sP)
{
    printMessage(6,"[vtRFThread::initRepresentativeTaxels] Initializing representative taxels for %s, version %s\n",sP.name.c_str(),sP.version.c_str());
    int j=0; //here j will start from 0 and correspond to taxel ID
    list<unsigned int> taxels_list;
    if (sP.name == SkinPart_s[SKIN_LEFT_FOREARM] || sP.name == SkinPart_s[SKIN_RIGHT_FOREARM])
    {
        for (j=0;j<sP.size;j++)
        {
            //4th taxel of each 12, but with ID 3, j starting from 0 here, is the triangle midpoint
            sP.taxel2Repr.push_back(((j/12)*12)+3); //initialize all 384 taxels with triangle center as the representative
            //fill a map of lists here somehow
        }

        // set to -1 the taxel2Repr for all the taxels that don't exist
        if (sP.version == "V1")
        {
            for (j=192;j<=203;j++)
            {
                sP.taxel2Repr[j]=-1; //these taxels don't exist
            }
        }
        for (j=216;j<=227;j++)
        {
            sP.taxel2Repr[j]=-1; //these taxels don't exist
        }
        if (sP.version == "V1")
        {
            for (j=228;j<=239;j++)
            {
                sP.taxel2Repr[j]=-1; //these taxels don't exist
            }
        }
        for (j=240;j<=251;j++)
        {
            sP.taxel2Repr[j]=-1; //these taxels don't exist
        }
        if (sP.version == "V2")
        {
            for (j=252;j<=263;j++)
            {
                sP.taxel2Repr[j]=-1; //these taxels don't exist
            }
        }
        if (sP.version == "V1")
        {
            for (j=264;j<=275;j++)
            {
                sP.taxel2Repr[j]=-1; //these taxels don't exist
            }
        }
        for (j=276;j<=287;j++)
        {
            sP.taxel2Repr[j]=-1; //these taxels don't exist
        }
        if (sP.version == "V2")
        {
            for (j=312;j<=323;j++)
            {
                sP.taxel2Repr[j]=-1; //these taxels don't exist
            }
        }
        for (j=324;j<=335;j++)
        {
            sP.taxel2Repr[j]=-1; //these taxels don't exist
        }
        for (j=360;j<=383;j++)
        {
            sP.taxel2Repr[j]=-1; //these taxels don't exist
        }

        // Set up the inverse - from every representative taxel to list of taxels it is representing
        taxels_list.clear();
        for(j=0;j<=11;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[3] = taxels_list;

        taxels_list.clear();
        for(j=12;j<=23;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[15] = taxels_list;

        taxels_list.clear();
        for(j=24;j<=35;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[27] = taxels_list;

        taxels_list.clear();
        for(j=36;j<=47;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[39] = taxels_list;

        taxels_list.clear();
        for(j=48;j<=59;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[51] = taxels_list;

        taxels_list.clear();
        for(j=60;j<=71;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[63] = taxels_list;

        taxels_list.clear();
        for(j=72;j<=83;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[75] = taxels_list;

        taxels_list.clear();
        for(j=84;j<=95;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[87] = taxels_list;

        taxels_list.clear();
        for(j=96;j<=107;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[99] = taxels_list;

        taxels_list.clear();
        for(j=108;j<=119;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[111] = taxels_list;

        taxels_list.clear();
        for(j=120;j<=131;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[123] = taxels_list;

        taxels_list.clear();
        for(j=132;j<=143;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[135] = taxels_list;

        taxels_list.clear();
        for(j=144;j<=155;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[147] = taxels_list;

        taxels_list.clear();
        for(j=156;j<=167;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[159] = taxels_list;

        taxels_list.clear();
        for(j=168;j<=179;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[171] = taxels_list;

        taxels_list.clear();
        for(j=180;j<=191;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[183] = taxels_list;
        //up to here - lower (full) patch on forearm

        //from here - upper patch with many dummy positions on the port - incomplete patch
        if (sP.version == "V2")
        { 
            taxels_list.clear();
            for(j=192;j<=203;j++)
            {
                taxels_list.push_back(j);
            }
            sP.repr2TaxelList[195] = taxels_list;
        }
        
        taxels_list.clear();
        for(j=204;j<=215;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[207] = taxels_list;

        if (sP.version == "V2")
        {
            taxels_list.clear();
            for(j=228;j<=237;j++)
            {
                taxels_list.push_back(j);
            }
            sP.repr2TaxelList[231] = taxels_list;
        }
        
        if (sP.version == "V1")
        {
            taxels_list.clear();
            for(j=252;j<=263;j++)
            {
                taxels_list.push_back(j);
            }
            sP.repr2TaxelList[255] = taxels_list;
        }
         
        if (sP.version == "V2")
        {
            taxels_list.clear();
            for(j=264;j<=275;j++)
            {
                taxels_list.push_back(j);
            }
            sP.repr2TaxelList[267] = taxels_list;
        }
        
        taxels_list.clear();
        for(j=288;j<=299;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[291] = taxels_list;

        taxels_list.clear();
        for(j=300;j<=311;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[303] = taxels_list;

        if (sP.version == "V1")
        {
            taxels_list.clear();
            for(j=312;j<=323;j++)
            {
                taxels_list.push_back(j);
            }
            sP.repr2TaxelList[315] = taxels_list;
        }
        
        taxels_list.clear();
        for(j=336;j<=347;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[339] = taxels_list;

        taxels_list.clear();
        for(j=348;j<=359;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[351] = taxels_list;
    }
    else if (sP.name == SkinPart_s[SKIN_LEFT_UPPER_ARM])
    {
        sP.taxel2Repr = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 123, 123, 123, 123, 123, 123, 123, 123, 123, 123, 123, 123, 135, 135, 135, 135, 135, 135, 135, 135, 135, 135, 135, 135, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 267, 267, 267, 267, 267, 267, 267, 267, 267, 267, 267, 267, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 291, 291, 291, 291, 291, 291, 291, 291, 291, 291, 291, 291, 303, 303, 303, 303, 303, 303, 303, 303, 303, 303, 303, 303, 315, 315, 315, 315, 315, 315, 315, 315, 315, 315, 315, 315, 327, 327, 327, 327, 327, 327, 327, 327, 327, 327, 327, 327, 339, 339, 339, 339, 339, 339, 339, 339, 339, 339, 339, 339, 351, 351, 351, 351, 351, 351, 351, 351, 351, 351, 351, 351, 363, 363, 363, 363, 363, 363, 363, 363, 363, 363, 363, 363, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 375, 387, 387, 387, 387, 387, 387, 387, 387, 387, 387, 387, 387, 399, 399, 399, 399, 399, 399, 399, 399, 399, 399, 399, 399, 411, 411, 411, 411, 411, 413, 413, 411, 411, 411, 411, 411, 423, 423, 423, 423, 423, 423, 423, 423, 423, 423, 423, 423, 435, 435, 435, 435, 435, 435, 435, 435, 435, 435, 435, 435, 447, 447, 447, 447, 447, 447, 447, 447, 447, 447, 447, 447, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, 471, 471, 471, 471, 471, 471, 471, 471, 471, 471, 471, 471, 483, 483, 483, 483, 483, 483, 483, 483, 483, 483, 483, 483, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 543, 543, 543, 543, 543, 543, 543, 543, 543, 543, 543, 543, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 579, 579, 579, 579, 579, 579, 579, 579, 579, 579, 579, 579, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 687, 687, 687, 687, 687, 687, 687, 687, 687, 687, 687, 687, 699, 699, 699, 699, 699, 699, 699, 699, 699, 699, 699, 699, 711, 711, 711, 711, 711, 711, 711, 711, 711, 711, 711, 711, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 735, 735, 735, 735, 735, 735, 735, 735, 735, 735, 735, 735, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 759, 759, 759, 759, 759, 759, 759, 759, 759, 759, 759, 759};

        // Set up the inverse - from every representative taxel to list of taxels it is representing
        auto indexes = std::vector<int>{39, 51, 63, 75, 87, 111, 123, 135, 195, 207, 267, 291, 303, 315,
                                        327, 339, 351, 363, 375, 387, 399, 411, 413, 423, 435, 447, 459,
                                        471, 483, 531, 543, 579, 639, 675, 687, 699, 711, 735, 759};

        for (auto& idx : indexes)
        {
            taxels_list.clear();
            for(j=idx-3;j<=idx+8;j++)
            {
                taxels_list.push_back(j);
            }
            sP.repr2TaxelList[idx] = taxels_list;
        }
    }
    else if (sP.name == SkinPart_s[SKIN_RIGHT_UPPER_ARM])
    {
        sP.taxel2Repr = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 123, 123, 123, 123, 123, 123, 123, 123, 123, 123, 123, 123, 135, 135, 135, 135, 135, 135, 135, 135, 135, 135, 135, 135, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 195, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 219, 219, 219, 219, 219, 219, 219, 219, 219, 219, 219, 219, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 267, 267, 267, 267, 267, 267, 267, 267, 267, 267, 267, 267, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 291, 291, 291, 291, 291, 291, 291, 291, 291, 291, 291, 291, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 339, 339, 339, 339, 339, 339, 339, 339, 339, 339, 339, 339, 351, 351, 351, 351, 351, 351, 351, 351, 351, 351, 351, 351, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 387, 387, 387, 387, 387, 387, 387, 387, 387, 387, 387, 387, 399, 399, 399, 399, 399, 399, 399, 399, 399, 399, 399, 399, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 483, 483, 483, 483, 483, 483, 483, 483, 483, 483, 483, 483, 495, 495, 495, 495, 495, 495, 495, 495, 495, 495, 495, 495, 507, 507, 507, 507, 507, 507, 507, 507, 507, 507, 507, 507, 519, 519, 519, 519, 519, 519, 519, 519, 519, 519, 519, 519, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 543, 543, 543, 543, 543, 543, 543, 543, 543, 543, 543, 543, 555, 555, 555, 555, 555, 555, 555, 555, 555, 555, 555, 555, 567, 567, 567, 567, 567, 567, 567, 567, 567, 567, 567, 567, 579, 579, 579, 579, 579, 579, 579, 579, 579, 579, 579, 579, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, 639, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 687, 687, 687, 687, 687, 687, 687, 687, 687, 687, 687, 687, 699, 699, 699, 699, 699, 699, 699, 699, 699, 699, 699, 699, 711, 711, 711, 711, 711, 711, 711, 711, 711, 711, 711, 711, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 735, 735, 735, 735, 735, 735, 735, 735, 735, 735, 735, 735, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 759, 759, 759, 759, 759, 759, 759, 759, 759, 759, 759, 759};

        // Set up the inverse - from every representative taxel to list of taxels it is representing
        auto indexes = std::vector<int>{39,  51,  63,  75,  87, 111, 123, 135, 195, 207, 219, 231,
                                        243, 255, 267, 279, 291, 339, 351, 387, 399, 459, 483, 495, 507,
                                        519, 531, 543, 555, 567, 579, 639, 675, 687, 699, 711, 735, 759};

        for (auto& idx : indexes)
        {
            taxels_list.clear();
            for(j=idx-3;j<=idx+8;j++)
            {
                taxels_list.push_back(j);
            }
            sP.repr2TaxelList[idx] = taxels_list;
        }
    }
    else if (sP.name == SkinPart_s[SKIN_FRONT_TORSO])
    {
        sP.taxel2Repr = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 75, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 87, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 111, 123, 123, 123, 123, 123, 123, 123, 123, 123, 123, 123, 123, 135, 135, 135, 135, 135, 135, 135, 135, 135, 135, 135, 135, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 207, 219, 219, 219, 219, 219, 219, 219, 219, 219, 219, 219, 219, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 267, 267, 267, 267, 267, 267, 267, 267, 267, 267, 267, 267, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 279, 291, 291, 291, 291, 291, 291, 291, 291, 291, 291, 291, 291, 303, 303, 303, 303, 303, 303, 303, 303, 303, 303, 303, 303, 315, 315, 315, 315, 315, 315, 315, 315, 315, 315, 315, 315, 327, 327, 327, 327, 327, 327, 327, 327, 327, 327, 327, 327, 339, 339, 339, 339, 339, 339, 339, 339, 339, 339, 339, 339, 351, 351, 351, 351, 351, 351, 351, 351, 351, 351, 351, 351, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 387, 387, 387, 387, 387, 387, 387, 387, 387, 387, 387, 387, 399, 399, 399, 399, 399, 399, 399, 399, 399, 399, 399, 399, 411, 411, 411, 411, 411, 411, 411, 411, 411, 411, 411, 411, 423, 423, 423, 423, 423, 423, 423, 423, 423, 423, 423, 423, 435, 435, 435, 435, 435, 435, 435, 435, 435, 435, 435, 435, 447, 447, 447, 447, 447, 447, 447, 447, 447, 447, 447, 447, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, 459, 471, 471, 471, 471, 471, 471, 471, 471, 471, 471, 471, 471, 483, 483, 483, 483, 483, 483, 483, 483, 483, 483, 483, 483, 495, 495, 495, 495, 495, 495, 495, 495, 495, 495, 495, 495, 507, 507, 507, 507, 507, 507, 507, 507, 507, 507, 507, 507, 519, 519, 519, 519, 519, 519, 519, 519, 519, 519, 519, 519, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 531, 543, 543, 543, 543, 543, 543, 543, 543, 543, 543, 543, 543, 555, 555, 555, 555, 555, 555, 555, 555, 555, 555, 555, 555, 567, 567, 567, 567, 567, 567, 567, 567, 567, 567, 567, 567, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 591, 591, 591, 591, 591, 591, 591, 591, 591, 591, 591, 591, 603, 603, 603, 603, 603, 603, 603, 603, 603, 603, 603, 603, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 627, 627, 627, 627, 627, 627, 627, 627, 627, 627, 627, 627, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 675, 687, 687, 687, 687, 687, 687, 687, 687, 687, 687, 687, 687, 699, 699, 699, 699, 699, 699, 699, 699, 699, 699, 699, 699, 711, 711, 711, 711, 711, 711, 711, 711, 711, 711, 711, 711, 723, 723, 723, 723, 723, 723, 723, 723, 723, 723, 723, 723, 735, 735, 735, 735, 735, 735, 735, 735, 735, 735, 735, 735, 747, 747, 747, 747, 747, 747, 747, 747, 747, 747, 747, 747, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

        // Set up the inverse - from every representative taxel to list of taxels it is representing
        auto indexes = std::vector<int>{63,  75,  87,  99, 111, 123, 135, 207, 219, 231, 267, 279, 291, 303, 315,
                                        327, 339, 351, 387, 399, 411, 423, 435, 447, 459, 471, 483, 495, 507, 519,
                                        531, 543, 555, 567, 591, 603, 627, 675, 687, 699, 711, 723, 735, 747};

        for (auto& idx : indexes)
        {
            taxels_list.clear();
            for(j=idx-3;j<=idx+8;j++)
            {
                taxels_list.push_back(j);
            }
            sP.repr2TaxelList[idx] = taxels_list;
        }
    }
    else if(sP.name == SkinPart_s[SKIN_LEFT_HAND])
    {
        for(j=0;j<sP.size;j++)
        {
            // Fill all the 192 with -1 - half of the taxels don't exist,
            // and for fingertips we don't have positions either
            sP.taxel2Repr.push_back(-1);
        }

        // Upper left area of the palm - at thumb
        for (j=121;j<=128;j++)
        {
            sP.taxel2Repr[j] = 122;
        }
        sP.taxel2Repr[131] = 122; //thermal pad

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear();
        for(j=121;j<=128;j++)
        {
            taxels_list.push_back(j);
        }
        taxels_list.push_back(131);
        sP.repr2TaxelList[122] = taxels_list;

        // Upper center of the palm
        for (j=96;j<=99;j++)
        {
            sP.taxel2Repr[j] = 99;
        }
        sP.taxel2Repr[102] = 99;
        sP.taxel2Repr[103] = 99;
        sP.taxel2Repr[120] = 99;
        sP.taxel2Repr[129] = 99;
        sP.taxel2Repr[130] = 99;

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear();
        for(j=96;j<=99;j++)
        {
            taxels_list.push_back(j);
        }
        taxels_list.push_back(102);
        taxels_list.push_back(103);
        taxels_list.push_back(120);
        taxels_list.push_back(129);
        taxels_list.push_back(130);
        sP.repr2TaxelList[99] = taxels_list;

        // Upper right of the palm (away from the thumb)
        sP.taxel2Repr[100] = 101;
        sP.taxel2Repr[101] = 101;
        for (j=104;j<=107;j++)
        {
            sP.taxel2Repr[j] = 101; //N.B. 107 is thermal pad
        }
        sP.taxel2Repr[113] = 101;
        sP.taxel2Repr[116] = 101;
        sP.taxel2Repr[117] = 101;

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear();
        taxels_list.push_back(100);
        taxels_list.push_back(101);
        for(j=104;j<=107;j++)
        {
            taxels_list.push_back(j);
        }
        taxels_list.push_back(113);
        taxels_list.push_back(116);
        taxels_list.push_back(117);
        sP.repr2TaxelList[101] = taxels_list;

        // Center area of the palm
        for(j=108;j<=112;j++)
        {
            sP.taxel2Repr[j] = 109;
        }
        sP.taxel2Repr[114] = 109;
        sP.taxel2Repr[115] = 109;
        sP.taxel2Repr[118] = 109;
        sP.taxel2Repr[142] = 109;
        sP.taxel2Repr[143] = 109;

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear();
        for(j=108;j<=112;j++)
        {
            taxels_list.push_back(j);
        }
        taxels_list.push_back(114);
        taxels_list.push_back(115);
        taxels_list.push_back(118);
        taxels_list.push_back(142);
        taxels_list.push_back(143);
        sP.repr2TaxelList[109] = taxels_list;

        // Lower part of the palm
        sP.taxel2Repr[119] = 134; // this one is thermal pad
        for(j=132;j<=141;j++)
        {
            sP.taxel2Repr[j] = 134;
        }

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear();
        taxels_list.push_back(119);
        for(j=132;j<=141;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[134] = taxels_list;

        for (unsigned int k = 200; k < 219; k++) {
            sP.taxel2Repr[k] = k;
            sP.repr2TaxelList[k] = list<unsigned int>{k};
        }
    }
    else if(sP.name == SkinPart_s[SKIN_RIGHT_HAND])
    {
       for(j=0;j<sP.size;j++)
       {
          sP.taxel2Repr.push_back(-1); //let's fill all the 192 with -1 - half of the taxels don't exist and for fingertips,
          //we don't have positions either
       }
       //upper left area - away from thumb on this hand
        sP.taxel2Repr[96] = 101;
        sP.taxel2Repr[97] = 101;
        sP.taxel2Repr[98] = 101;
        sP.taxel2Repr[100] = 101;
        sP.taxel2Repr[101] = 101;
        sP.taxel2Repr[107] = 101; //thermal pad
        sP.taxel2Repr[110] = 101;
        sP.taxel2Repr[111] = 101;
        sP.taxel2Repr[112] = 101;

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear();
        taxels_list.push_back(96);
        taxels_list.push_back(97);
        taxels_list.push_back(98);
        taxels_list.push_back(100);
        taxels_list.push_back(101);
        taxels_list.push_back(107);
        taxels_list.push_back(110);
        taxels_list.push_back(111);
        taxels_list.push_back(112);
        sP.repr2TaxelList[101] = taxels_list;

        //upper center of the palm
        sP.taxel2Repr[99] = 103;
        for(j=102;j<=106;j++)
        {
           sP.taxel2Repr[j] = 103;
        }
        sP.taxel2Repr[127] = 103;
        sP.taxel2Repr[129] = 103;
        sP.taxel2Repr[130] = 103;

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear();
        taxels_list.push_back(99);
        for(j=102;j<=106;j++)
        {
            taxels_list.push_back(j);
        }
        taxels_list.push_back(127);
        taxels_list.push_back(129);
        taxels_list.push_back(130);
        sP.repr2TaxelList[103] = taxels_list;


        //upper right center of the palm - at thumb
        for(j=120;j<=126;j++)
        {
           sP.taxel2Repr[j] = 124;
        }
        sP.taxel2Repr[128] = 124;
        sP.taxel2Repr[131] = 124; //thermal pad

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear();
        for(j=120;j<=126;j++)
        {
            taxels_list.push_back(j);
        }
        taxels_list.push_back(128);
        taxels_list.push_back(131);
        sP.repr2TaxelList[124] = taxels_list;


        //center of palm
        sP.taxel2Repr[108] = 118;
        sP.taxel2Repr[109] = 118;
        for(j=113;j<=118;j++)
        {
            sP.taxel2Repr[j] = 118;
        }
        sP.taxel2Repr[142] = 118;
        sP.taxel2Repr[143] = 118;

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear();
        taxels_list.push_back(108);
        taxels_list.push_back(109);
        for(j=113;j<=118;j++)
        {
            taxels_list.push_back(j);
        }
        taxels_list.push_back(142);
        taxels_list.push_back(143);
        sP.repr2TaxelList[118] = taxels_list;

        //lower palm
        sP.taxel2Repr[119] = 137; //thermal pad
        for(j=132;j<=141;j++)
        {
            sP.taxel2Repr[j] = 137; //139 is another thermal pad
        }

        // Set up the mapping in the other direction - from every representative taxel to list of taxels it is representing
        taxels_list.clear();
        taxels_list.push_back(119);
        for(j=132;j<=141;j++)
        {
            taxels_list.push_back(j);
        }
        sP.repr2TaxelList[137] = taxels_list;
        for (unsigned int k = 200; k < 219; k++) {
            sP.taxel2Repr[k] = k;
            sP.repr2TaxelList[k] = list<unsigned int>{k};
        }
    }
}

bool vtRFThread::getRepresentativeTaxels(const std::vector<unsigned int>& IDv, const int IDx, std::vector<unsigned int> &v)
{
    //unordered_set would be better, but that is only experimentally supported by some compilers.
    std::set<unsigned int> rep_taxel_IDs_set;

    if (iCubSkin[IDx].taxel2Repr.empty())
    {
        v = IDv; //we simply copy the activated taxels
        return false;
    }
    else
    {
        for (unsigned int it : IDv)
        {
            if (iCubSkin[IDx].taxel2Repr[it] == -1)
            {
                yWarning("[%s] taxel %u activated, but representative taxel undefined - ignoring.",iCubSkin[IDx].name.c_str(),it);
            }
            else
            {
                rep_taxel_IDs_set.insert(iCubSkin[IDx].taxel2Repr[it]); //add all the representatives that were activated to the set
            }
        }

        for (unsigned int itr : rep_taxel_IDs_set)
        {
            v.push_back(itr); //add the representative taxels that were activated to the output taxel ID vector
        }

        if (v.empty())
        {
            yWarning("Representative taxels' vector is empty! Skipping.");
            return false;
        }

        if (verbosity>=4)
        {
            printMessage(4,"Representative taxels on skin part %d: ",IDx);
            for(unsigned int it : v)
            {
                printf("%d ",it);
            }
            printf("\n");
        }
    }

    return true;
}

int vtRFThread::printMessage(const int l, const char *f, ...) const
{
    if (verbosity>=l)
    {
        fprintf(stdout,"[%s] ",name.c_str());

        va_list ap;
        va_start(ap,f);
        int ret=vfprintf(stdout,f,ap);
        va_end(ap);

        return ret;
    }
    else
        return -1;
}

void vtRFThread::threadRelease()
{
    yDebug("[vtRF::threadRelease]Saving taxels..\n");
        save();

    yDebug("[vtRF::threadRelease]Closing controllers..\n");
        ddR.close();
        ddL.close();
        ddT.close();
        ddH.close();

    yDebug("[vtRF::threadRelease]Deleting misc stuff..\n");
        delete armR;
        armR = nullptr;
        delete armL;
        armL = nullptr;
        delete torso;
        torso = nullptr;
        delete eWR;
        eWR  = nullptr;
        delete eWL;
        eWL  = nullptr;

    yDebug("[vtRF::threadRelease]Closing ports..\n");
        closePort(imagePortInR);
        yDebug("  imagePortInR successfully closed!\n");
        closePort(imagePortInL);
        yDebug("  imagePortInL successfully closed!\n");

        // closePort(imagePortOutR);
        imagePortOutR.interrupt();
        imagePortOutR.close();
        yDebug("  imagePortOutR successfully closed!\n");
        // closePort(imagePortOutL);
        imagePortOutL.interrupt();
        imagePortOutL.close();
        yDebug("  imagePortOutL successfully closed!\n");

        closePort(dTPort);
        yDebug("  dTPort successfully closed!\n");

        closePort(stressPort);
        yDebug("    stressPort successfully closed!\n");

        closePort(eventsPort);
        yDebug("  eventsPort successfully closed!\n");

        closePort(skinPortIn);
        yDebug("  skinPortIn successfully closed!\n");

        ppsEventsPortOut.interrupt();
        ppsEventsPortOut.close();
        yDebug("ppsEventsPortOut successfully closed!\n");

        // closePort(skinGuiPortForearmL);
        skinGuiPortForearmL.interrupt();
        skinGuiPortForearmL.close();
        yDebug("  skinGuiPortForearmL successfully closed!\n");
        skinGuiPortUpperarmL.interrupt();
        skinGuiPortUpperarmL.close();
        yDebug("  skinGuiPortUpperarmL successfully closed!\n");
        skinGuiPortUpperarmR.interrupt();
        skinGuiPortUpperarmR.close();
        yDebug("  skinGuiPortUpperarmR successfully closed!\n");
        skinGuiPortTorso.interrupt();
        skinGuiPortTorso.close();
        yDebug("  skinGuiPortTorso successfully closed!\n");
    // closePort(skinGuiPortForearmR);
        skinGuiPortForearmR.interrupt();
        skinGuiPortForearmR.close();
        yDebug("  skinGuiPortForearmR successfully closed!\n");
        // closePort(skinGuiPortHandL);
        skinGuiPortHandL.interrupt();
        skinGuiPortHandL.close();
        yDebug("  skinGuiPortHandL successfully closed!\n");
        // closePort(skinGuiPortHandR);
        skinGuiPortHandR.interrupt();
        skinGuiPortHandR.close();
        yDebug("  skinGuiPortHandR successfully closed!\n");
    yInfo("[vtRF::threadRelease] done.\n");
}

// empty line to make gcc happy
