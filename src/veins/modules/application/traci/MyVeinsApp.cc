/*
* Author            : Marco Rapelli & Ahmadreza Jame
* Date              : 2019-2020
* Purpose           : Create a baseline for Vehicle-to-Vehicle simulated application
*/

#include "veins/modules/application/traci/MyVeinsApp.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
simtime_t temptime,temptime2;
int tempcount=0;
int tempStopNGo=0;
int shedCount=0;

using namespace veins;

Define_Module(veins::MyVeinsApp);


/**
 * The initialize function is called by every vehicle just inserted in the map.
 * It is mainly used for initializing pointers and main variables.
 */
int i=0;
void MyVeinsApp::initialize(int stage)
{
    DemoBaseApplLayer::leaderFlag=false;
    DemoBaseApplLayer::intersectionFlag=false;
    DemoBaseApplLayer::scheduledFlag=false;
    this->storage.legalMoves();


    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        // Initializing members and pointers of your application goes here
        EV << "Initializing " << par("appName").stringValue() << std::endl;

        //initializing the mobility environment
        mobility=TraCIMobilityAccess().get(getParentModule());
        traci=mobility->getCommandInterface();
        traciVehicle=mobility->getVehicleCommandInterface();

        //initializing void VehicleData and CarStorage occurrences
        this->vehicle=VehicleData();
        this->storage=CarStorage();

        //initializing the old threshold
        oldThreshold=par("oldThreshold").doubleValue();
    }
    else if (stage == 1) {
        // Initializing members that require initialized other modules goes here
    }
}

/**
 * The finish function is called by every vehicle which has just finished its route.
 * It is mainly used for recording statistics (e.g., number of running vehicles, number of exited vehicles, etc.).
 */
void MyVeinsApp::finish()
{  // if(vehId==396)
   // std::cout<<simTime()<<endl;
    //std::cout<<vehId<<endl;
    DemoBaseApplLayer::finish();


    // statistics recording goes here
}

/**
 * The onBSM is called upon a reception of a Basic Safety Message.
 * BSMs are used to disseminate basic motion information (e.g, id, position, speed, etc.) in order to let neighbours know the presence of each vehicle.
 * The reception of a BSM usually triggers the main V2V application (after the update of the sender information in the storage).
 */
//NOTE: to change the message type if an other type of message is used!!
//NOTE: to change the message getters fields if an other type of message is used!!
void MyVeinsApp::onBSM(BasicSafetyMessage* bsm)
{
    // Your application has received a beacon message from another car or RSU
    // code for handling the message goes here

    //extract message information
    unsigned int msgID=bsm->getMsgID();
    unsigned int msgCount=bsm->getMsgCount();
    unsigned int msgPriority=bsm->getMsgPriority();

    //extract sender information
    unsigned int senderID=bsm->getSenderID();
    simtime_t sendingTime=bsm->getSendingTime();
    Coord senderPosition=bsm->getSenderPosition();
    Coord senderVelocity=bsm->getSenderVelocity();
    double senderDirection=bsm->getSenderDirection();
    std::string senderSignal=bsm->getSenderSignal();


    double senderLength=bsm->getSenderLength();
    double senderWidth=bsm->getSenderWidth();
    double senderDistance=bsm->getSenderDistance();
    std::string senderLeaderString=bsm->getSenderLeaderString();
    std::string senderIntersectionString=bsm->getSenderIntersectionString();
    std::string senderSolutionDataset=bsm->getSenderSolutionDataset();
    bool    snederIntersectionFlag=bsm->getSenderIntersectionFlag();
    bool    senderScheduledFlag=bsm->getSenderScheduledFlag();
    bool    senderLeaderElectionFlag=bsm->getSenderLeaderElectionFlag();
    int senderPositionInQueue=bsm->getSenderPositionInQueue();
    std::string roadId=bsm->getSenderRoadId();
    int senderTurn=bsm->getSenderTurn();
//only leaders should save the leader string due to memory saving
    if(this->vehicle.getLeaderElectionFlg()==true)
        senderLeaderString=bsm->getSenderLeaderString();



//if(this->vehicle.getLeaderElectionFlg()==true && senderScheduledFlag==true )

//the first car who calculate the solution share with other leaders
//{ this->vehicle.setScheduledFlg(true);
//   DemoBaseApplLayer::scheduledFlag=true;
//   this->vehicle.setSolution(senderSolutionDataset);
//   DemoBaseApplLayer::solutionDataset=senderSolutionDataset;
//   std::cout<<senderSolutionDataset;
//}


    //create a VehicleData with sender information
    VehicleData sender=VehicleData(sendingTime,senderID,senderPosition,std::make_pair(senderVelocity.x,senderVelocity.y),senderDirection,senderSignal,senderLength,senderWidth,

            senderDistance,senderLeaderString,senderIntersectionString,senderSolutionDataset,senderLeaderElectionFlag,snederIntersectionFlag,senderScheduledFlag,roadId,senderPositionInQueue,senderTurn,0);

        this->storage.updateStorage(sender,oldThreshold);

        }



/**
 * The onWhereAmI function is called upon a WhereAmI event.
 * WhereAmI events are standard events generated internally by each vehicle, so they usually do not trigger any message sending.
 * They are used to update the self mobility information in the self storage.
 */
void MyVeinsApp::onWhereAmI()
{
    // Your application has received a whereAmI event from itself
    // code for handling the event goes here

    //extract personal information
    double distance=-1;
    simtime_t timestamp=simTime();
    Coord intersect;
    intersect=traci->junction("5").getPosition();
    Coord position=this->mobility->getPositionAt(timestamp);
    double direction=this->mobility->getHeading().getRad();
    double velocityX=cos(direction)*this->mobility->getSpeed();
    double velocityY=sin(direction)*this->mobility->getSpeed();
    if(abs(velocityX)<abs(0.0001))
        velocityX=0;
    if(abs(velocityY)<abs(0.0001))
        velocityY=0;
    std::pair<double,double> velocity=std::make_pair(velocityX,velocityY);
    std::string blinkingLight=this->traciVehicle->getRouteId();
    double vehLength=this->traciVehicle->getLength();
    double vehWidth=this->traciVehicle->getWidth();
    std::string leaderStr=this->vehicle.getLeader();
    std::string intersectionStr=this->vehicle.getIntersectionStr();
    std::string solutionDataset=this->vehicle.getSolution();
    bool  interFlag=this->vehicle.getIntersectionflg();
    bool  schedulFlag=this->vehicle.getScheduledFlg();
    bool leaderFlag=this->vehicle.getLeaderElectionFlg();
    int positionInQueue=this->vehicle.getPositionInQueue();
    distance=position.distance(intersect);
    DemoBaseApplLayer::vehDistance=distance;
    double vehDistance=distance;
    std::string roadId=this->traciVehicle->getRoadId();
    DemoBaseApplLayer::roadid=roadId;
    int turn=this->vehicle.getTurn();
    double lights=this->mobility->getSignals().to_ulong();



    if(this->storage.isPassedIntersection(vehId,/*distance ))*/ roadId))
    {
        interFlag=true;
        DemoBaseApplLayer::intersectionFlag=true;
        leaderFlag=false;
        DemoBaseApplLayer::leaderFlag=false;
        positionInQueue=0;
        DemoBaseApplLayer::positionInQueue=0;
       // schedulFlag=false;
        //DemoBaseApplLayer::scheduledFlag=false;
        //this->traciVehicle->setSpeed(100);
        turn=-1;
        DemoBaseApplLayer::turn=-1;



    }
    if(this->vehicle.getVel().first<.01 && velocityX>=.01)
                  tempStopNGo++;

//    if(distance<100 && this->vehicle.getLight()>0 && lights>0)
//        if(lights!=this->vehicle.getLight())
//            tempStopNGo++;


if(interFlag!=this->vehicle.getIntersectionflg())
    tempcount++;

if((simTime()-temptime)>=60)
           { this->storage.recorder(tempcount,tempStopNGo,shedCount);
               tempcount=0;
               //std::cout<<tempStopNGo<<endl;
               tempStopNGo=0;
               shedCount=0;
               temptime=simTime();
           }



if(distance<50)
   this->traciVehicle->setSpeed(5);





    Nc=par("Nc");



// setting Queue Positions of each car vehdistance for costnant leaders
//members actively search to update theis position  before the getting close to the intersection, after that positions and groups remain the same for scheduling
if(interFlag==false && schedulFlag==false && distance>30 ) //&& vehDistance>900)
{
        int Carposition=this->storage.QpositionSeter(vehId, vehDistance,direction,schedulFlag);
//        if(vehId==0)
//            std::cout<<Carposition<<endl;
        if((Carposition==-1) || (Carposition==Nc))
        {   //std::cout<<"position of this is "<<vehId<<endl;
            positionInQueue=1;
            DemoBaseApplLayer::positionInQueue=1;
        }

        else if(Carposition<Nc && Carposition!=-2)
        {
            positionInQueue=++Carposition;
            DemoBaseApplLayer::positionInQueue=positionInQueue;

        }
//        else
//        {
//            positionInQueue=-2;
//            DemoBaseApplLayer::positionInQueue=-2;
//
//        }

}





//leader election prodedure
    if(positionInQueue==1 && vehDistance<200 && schedulFlag==false)
    { if(this->storage.leaderElection(vehId,direction, vehDistance, Nc,positionInQueue))
            {
             leaderFlag=true;
             DemoBaseApplLayer::leaderFlag=true;
            // std::cout<<"position of this is "<<vehId<<endl;
            }
        else
        {
            leaderFlag=false;
            DemoBaseApplLayer::leaderFlag=false;
        }
    }
    else
    { leaderFlag=false;
        DemoBaseApplLayer::leaderFlag=false;
    }








//leader string procedure
    if(leaderFlag==true)
             {
               leaderStr=this->storage.leaderString(vehId, direction,vehDistance,Nc);
               DemoBaseApplLayer::leaderString=leaderStr;
             }
    else
       {
        leaderStr="";
        DemoBaseApplLayer::leaderString=leaderStr;
       }

    if( leaderFlag==true && leaderStr!="" && distance<100)
        {
        intersectionStr=this->storage.intersectionString(vehId,direction,vehDistance);
        DemoBaseApplLayer::intersectionString=intersectionStr;

        }
    else
    {
        intersectionStr="";
        DemoBaseApplLayer::intersectionString="";
    }





//if(this->storage.stopCaller()==true && leaderFlag==true)
   // this->traciVehicle->stopAt(roadId, 990, this->traciVehicle->getLaneIndex(),this->mobility->getHeading().getRad(),10);










//scheduling process
    if(leaderFlag==true && schedulFlag==false && interFlag==false && intersectionStr!="" )
    {  // std::cout<<simTime()<<endl;
        solutionDataset=this->storage.solutionFinder(intersectionStr,vehId,Nc,direction);
        if(solutionDataset!="")
            {
            DemoBaseApplLayer::solutionDataset=solutionDataset;
            schedulFlag=true;
            DemoBaseApplLayer::scheduledFlag=true;
            shedCount++;
            //this->traciVehicle->stopAt(roadId, 995, this->traciVehicle->getLaneIndex(),this->mobility->getHeading().getRad(), 1);
            //this->traciVehicle->setSpeed(13.3);
            //std::cout<<vehId<<endl;
            //std::cout<<simTime()<<"\n\n";
            }

    }





    if(interFlag!=true && schedulFlag!=true)
           {
        solutionDataset=this->storage.solutionSetter(vehId, direction, vehDistance);
            if(solutionDataset!="")
            {
                schedulFlag=true;
                DemoBaseApplLayer::scheduledFlag=true;
                shedCount++;
                //DemoBaseApplLayer::solutionDataset=solutionDataset;
                //this->traciVehicle->stopAt(roadId, 995, this->traciVehicle->getLaneIndex(),this->mobility->getHeading().getRad(), 3);

                //this->traciVehicle->setSpeed(20);

//               i++;
              // std::cout<<vehId<<endl;

            }

           }

if(schedulFlag==true)

{
    std::string line;
    std::istringstream string(solutionDataset);
    int i=0;
    while(std::getline(string, line))
               {
                  int carId=std::stoi(line);


                   if(vehId==carId)
                     {
                       //std::cout<<"\n\n id "<<id<<endl;
                       turn=i/4;
                       DemoBaseApplLayer::turn=turn;
                       break;

                     }
                   i++;
               }

}


//forcing cars to move considering their level in solution data set
if(vehDistance<20)
    {

        if(this->storage.moveAllower(vehId, turn,roadId,distance,interFlag)==false)

           {
             //std::cout<<turn<<"id "<<vehId<<endl;
            //if(distance>10)
            //this->traciVehicle->setSpeed(0);



            if(distance>15)
            this->traciVehicle->stopAt(roadId, 995, this->traciVehicle->getLaneIndex(),this->mobility->getHeading().getRad(), .5);

            //this->traciVehicle->stopAt(roadId, intersect.x, this->traciVehicle->getLaneIndex(),this->mobility->getHeading().getRad(), 1);


        }
//        else
//          this->traciVehicle->setSpeed(13.3);

    }






//   if(vehId>35 && vehId<40)
//       this->storage.printStorageById(this->vehicle.getId());


//if(vehDistance<100)
   // std::cout<<this->traciVehicle->getRouteId()<<endl;



//create a VehicleData with self information
this->vehicle=VehicleData(timestamp,vehId,position,velocity,direction,blinkingLight,vehLength,vehWidth,vehDistance,leaderStr
        ,intersectionStr,solutionDataset,leaderFlag,interFlag,schedulFlag,roadId,positionInQueue,turn,lights);
//update the storage with the new data
    this->storage.updateStorage(this->vehicle,oldThreshold);












}
/**
 * The onWSM function is called upon a reception of a WAVE Short Message.
 * WSMs are usually sent as infotainment messages (e.g, bad weather conditions, data streaming, etc.).
 */
void MyVeinsApp::onWSM(BaseFrame1609_4* wsm)
{
    // Your application has received a data message from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples

    EV << "Received a WSM with the timestamp: " << wsm->getSendingTime() << std::endl;
}

/**
 * The onWSA function is called upon a reception of a WAVE Service Advertisement.
 * WSAs are sent as alerts in case of an hazardous situation (e.g, car accident forecasted or occurred, stationary vehicle, etc.).
 */
void MyVeinsApp::onWSA(DemoServiceAdvertisment* wsa)
{
    // Your application has received a service advertisement from another car or RSU
    // code for handling the message goes here, see TraciDemo11p.cc for examples

    EV << "Received a WSA with the timestamp: " << wsa->getSendingTime() << std::endl;
}

/**
 * The handleSelfMsg function is called upon a self message (usually an event timer).
 * It is often unused. For dealing with new event timers go to the handleSelfMsg of DemoBaseApplLayer.
 */
void MyVeinsApp::handleSelfMsg(cMessage* msg)
{
    DemoBaseApplLayer::handleSelfMsg(msg);
    // this method is for self messages (mostly timers)
    // it is important to call the DemoBaseApplLayer function for BSM and WSM transmission
}

/**
 * The handlePositionUpdate function is called upon the vehicle movings.
 * It can be used for applications reacting to car moves (e.g, applications relying on on-board sensors).
 */
void MyVeinsApp::handlePositionUpdate(cObject* obj)
{
    DemoBaseApplLayer::handlePositionUpdate(obj);
    // the vehicle has moved. Code that reacts to new positions goes here.
    // member variables such as currentPosition and currentSpeed are updated in the parent class DemoBaseApplLayer
}




