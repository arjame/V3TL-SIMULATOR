/*
* Author            : Marco Rapelli
* Date              : December 2019
* Purpose           : Create a vehicle data structure
*/

#include "veins/modules/application/data_structure/VehicleData.h"
#include<string.h>

using namespace veins;

/**
 * Getters and setters methods of VehicleData
 */
simtime_t VehicleData::getTimestamp() {
    return timestamp;
}

int VehicleData::getId() {
    return vehID;
}

Coord VehicleData::getPos() {
    return position;
}

std::pair<double,double> VehicleData::getVel() {
    return velocity;
}

double VehicleData::getDirect() {
    return direction;
}

std::string VehicleData::getLights() {
    return blinkingLight;
}

double VehicleData::getLength() {
    return vehLength;
}

double VehicleData::getWidth() {
    return vehWidth;
}
double VehicleData::getDistance() {
    return vehDistance;
}
std::string VehicleData::getIntersectionStr(){
    return intersectionString;
}
 std::string VehicleData::getLeader(){
    return leaderString;
}
std::string VehicleData::getSolution(){
    return solutionDataset;
}
bool VehicleData::getIntersectionflg(){
    return intersectionFlag;
}
bool VehicleData::getLeaderElectionFlg(){
    return leaderElectionFlag;
}
bool VehicleData::getScheduledFlg(){
    return scheduledFlag ;
}
std::string VehicleData::getRoadId(){
    return RoadId;
}
int VehicleData::getPositionInQueue()
{
    return PositionInQueue;
}
int VehicleData::getTurn()
{
    return turn;
}
double VehicleData::getLight()
{
    return lights;

}
void VehicleData::setLeaderElectionFlg(bool leadrFlg)
{leaderElectionFlag=leadrFlg;

}

void VehicleData::setLeaderString(std::string leaderSt)
{leaderString=leaderSt;

}


void VehicleData::setIntersectionStr(std::string intersectionSt)
{intersectionString=intersectionSt;

}

void VehicleData::setSolution(std::string solData)
{solutionDataset=solData;

}

void VehicleData::setIntersectionflg(bool interFlg)
{intersectionFlag=interFlg;

}
void VehicleData::setScheduledFlg(bool schlFlg)
{scheduledFlag=schlFlg;

}
void VehicleData::setRoadId(std::string roadid)
{RoadId=roadid;

}
void VehicleData::setTurn(int trn)
{
turn=trn;

}
void VehicleData::setLight(double lghts)
{
    lights=lghts;
}
/**
 * Update this VehicleData instance with the new one
 */
VehicleData VehicleData::update(VehicleData up) {

    timestamp=up.timestamp;
    vehID=up.vehID;
    position=up.position;
    velocity=up.velocity;
    direction=up.direction;
    blinkingLight=up.blinkingLight;
    vehLength=up.vehLength;
    vehWidth=up.vehWidth;
    vehDistance=up.vehDistance;
    intersectionString=up.intersectionString;
    leaderString=up.leaderString;
    solutionDataset=up.solutionDataset;
    intersectionFlag=up.intersectionFlag;
    leaderElectionFlag=up.leaderElectionFlag;
    scheduledFlag=up.scheduledFlag;
    RoadId=up.RoadId;
    PositionInQueue=up.PositionInQueue;
    turn=up.turn;
    lights=up.lights;



    return VehicleData(this->timestamp,this->vehID,this->position,this->velocity,this->direction,this->blinkingLight,this->vehLength,this->vehWidth
     ,this->vehDistance,this->leaderString,this->intersectionString,this->solutionDataset,this->leaderElectionFlag,this->intersectionFlag,this->scheduledFlag,this->RoadId,this->PositionInQueue,this->turn,this->lights);
}
