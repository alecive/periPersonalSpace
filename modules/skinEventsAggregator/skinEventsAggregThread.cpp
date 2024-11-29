#include <algorithm>
#include "skinEventsAggregThread.h"

#define MIN_NUMBER_OF_TAXELS_ACTIVATED 3 //default: 3; to filter out phantoms
#define SKIN_ACTIVATION_MAX_ICUB_SIM 100
#define SKIN_ACTIVATION_MAX_ICUB 30

using namespace yarp::os;
using namespace yarp::sig;

using namespace iCub::skinDynLib;


/*************** public methods section ********************************************/

skinEventsAggregThread::skinEventsAggregThread(int _rate, const string &_name, const string &_robot, int _v) : 
                                               PeriodicThread((double)_rate/1000.0),name(_name), robot(_robot), verbosity(_v) 
{

}

bool skinEventsAggregThread::threadInit()
{
    ts.update();
    skinEventsPortIn.open("/"+name+"/skin_events:i");
    skinEvAggregPortOut.open("/"+name+"/skin_events_aggreg:o");
   
    if (robot == "icub")
        SKIN_ACTIVATION_MAX = SKIN_ACTIVATION_MAX_ICUB;
    else if (robot == "icubSim")
        SKIN_ACTIVATION_MAX = SKIN_ACTIVATION_MAX_ICUB_SIM;
    else
        yError("skinEventsAggregThread::threadInit(): unknown robot type");
    
    return true;
}

void skinEventsAggregThread::run()
{
    int indexOfBiggestContact = -1;
    skinContact biggestContactInSkinPart;
    Vector geoCenter(3,0.0), normalDir(3,0.0);
    double activation = 0.0;
    Bottle & out = skinEvAggregPortOut.prepare(); out.clear();
    Bottle b;
    b.clear();
        
    ts.update();
        
    skinContactList *scl = skinEventsPortIn.read(false);
        
    if(scl)
    {
        if(!(scl->empty()))
        {  
            //Probably source of crazy inefficiencies, here just to reach a working state as soon as possible \todo TODO
            map<SkinPart, skinContactList> contactsPerSkinPart = scl->splitPerSkinPart();
            
            for(map<SkinPart,skinContactList>::iterator it=contactsPerSkinPart.begin(); it!=contactsPerSkinPart.end(); it++)
            {
                indexOfBiggestContact = getIndexOfBiggestContactInList(it->second);
                
                if (indexOfBiggestContact != -1)
                {
                    b.clear();
                    biggestContactInSkinPart = (it->second)[indexOfBiggestContact];
                    //the output prepared should have identical format to the one prepared in  void vtRFThread::manageSkinEvents()    
                    b.addInt32(biggestContactInSkinPart.getSkinPart());
                    vectorIntoBottle(biggestContactInSkinPart.getGeoCenter(),b);
                    vectorIntoBottle(biggestContactInSkinPart.getNormalDir(),b);
                    //we add dummy geoCenter and normalDir in Root frame to keep same format as vtRFThread manageSkinEvents 
                    b.addFloat64(0.0); b.addFloat64(0.0); b.addFloat64(0.0);
                    b.addFloat64(0.0); b.addFloat64(0.0); b.addFloat64(0.0);
                    b.addFloat64(std::max(1.0,(biggestContactInSkinPart.getPressure()/SKIN_ACTIVATION_MAX))); // % pressure "normalized" with ad hoc constant
                    b.addString(biggestContactInSkinPart.getSkinPartName()); //this one just for readability
                    out.addList().read(b);
                }
            }
            skinEvAggregPortOut.setEnvelope(ts);
            skinEvAggregPortOut.write();     // send something anyway (if there is no contact the bottle is empty)
      }
    }
}
  

void skinEventsAggregThread::threadRelease()
{
    printMessage(0,"Closing ports..\n");
    skinEventsPortIn.interrupt();
    skinEventsPortIn.close();
    printMessage(1,"skinEventPortIn successfully closed!\n");
    skinEvAggregPortOut.interrupt();
    skinEvAggregPortOut.close();
    printMessage(1,"skinEvAggregPortOut successfully closed!\n");
}

int skinEventsAggregThread::getIndexOfBiggestContactInList(iCub::skinDynLib::skinContactList &sCL)
{
    int index = -1;
    unsigned int maxActivatedTaxels = MIN_NUMBER_OF_TAXELS_ACTIVATED;
    if (! sCL.empty())
    {
        for(skinContactList::iterator c=sCL.begin(); c!=sCL.end(); c++)
        {
            printf("Link number is %d and skin number %d\n", c->getLinkNumber(), c->getSkinPart());
            bool skip = false;
            if( c->getActiveTaxels() >= maxActivatedTaxels && yarp::math::norm(c->getNormalDir()) > 0.8)
            {
                if (c->getSkinPart() == iCub::skinDynLib::SKIN_LEFT_HAND || c->getSkinPart() == iCub::skinDynLib::SKIN_RIGHT_HAND) {
                    for (unsigned int taxel: c->getTaxelList()) {
                        if (taxel < 70) {
                            std::cout << "Skip because " << taxel << " " << index << "\n";
                            skip = true;
                            break;
                        }
                    }
                }
                if (skip) continue;
                maxActivatedTaxels = c->getActiveTaxels();
                index = std::distance( sCL.begin(), c);
            }
        }
    }
    //if (index == -1)
      // yError("skinEventsAggregThread::getIndexOfBiggestContactInList: returning index -1");
    
    return index;
}
     
int skinEventsAggregThread::printMessage(const int l, const char *f, ...) const
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
