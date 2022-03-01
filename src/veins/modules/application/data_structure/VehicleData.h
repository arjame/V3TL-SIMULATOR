/*
* Author            : Marco Rapelli && Ahmadreza Jame
* Date              : 2019-2020
* Purpose           : Create a vehicle data structure
*/

#pragma once

#include "veins/veins.h"

#include <map>

#include "veins/base/utils/Coord.h"
#include <iostream>

using namespace omnetpp;

namespace veins {
    class VehicleData {

        protected:
            //VehicleData parameters
            simtime_t timestamp;
            int vehID;
            Coord position;
            std::pair<double,double> velocity;
            double direction;
            std::string  blinkingLight;
            double vehLength;
            double vehWidth;
            double vehDistance=-1;
            std::string leaderString;
            std::string intersectionString;
            std::string solutionDataset;
            bool intersectionFlag;
            bool scheduledFlag;
            bool leaderElectionFlag;
            std::string RoadId;
            int PositionInQueue;
            int turn=-1;
            double lights;





        public:
            //void instance of VehicleData
            VehicleData() {
                this->timestamp=-1;
                this->vehID=-1;
                this->position=Coord(-1,-1,-1);
                this->velocity=std::make_pair(-1,-1);
                this->direction=-1;
                this->blinkingLight="";
                this->vehLength=-1;
                this->vehWidth=-1;
                this->vehDistance=-1;
                this->leaderString="";
                this->intersectionString="";
                this->solutionDataset="";
                this->RoadId="";
                this->PositionInQueue=-1;
                this->intersectionFlag=false;
                this->scheduledFlag=false;
                this->leaderElectionFlag=false;
                this->turn=-1;
                this->lights=0;


            }


            //constructor of VehicleData
            VehicleData(simtime_t time, int id, Coord pos, std::pair<double,double> vel, double direct, std::string lights, double length, double width
                   ,double distance,std::string leaderSt,std::string intersectionSt,std::string solData,bool leadrFlg,bool interFlg,bool schlFlg,std::string roadid,int posInQ,int trn,double lghts) {
                this->timestamp=time;
                this->vehID=id;
                this->position=pos;
                this->velocity=vel;
                this->direction=direct;
                this->blinkingLight=lights;
                this->vehLength=length;
                this->vehWidth=width;
                this->vehDistance=distance;
                this->leaderString=leaderSt;
                this->intersectionString=intersectionSt;
                this->solutionDataset=solData;
                this->intersectionFlag=interFlg;
                this->scheduledFlag=schlFlg;
                this->leaderElectionFlag=leadrFlg;
                this->RoadId=roadid;
                this->PositionInQueue=posInQ;
                this->turn=trn;
                this->lights=lghts;


            }

            //destructor of VehicleData
            ~VehicleData() {

            }

            //VehicleData getters and setters methods
            simtime_t getTimestamp();
            int getId();
            Coord getPos();
            std::pair<double,double> getVel();
            double getDirect();
            std::string getLights();
            double getLength();
            double getWidth();
            double getDistance();
            virtual std::string getLeader() ;
            std::string getIntersectionStr();
            std::string getSolution();
            std::string getRoadId();
            int getPositionInQueue();
            void setRoadId(std::string roadid);
            bool getIntersectionflg();
            bool getScheduledFlg();
            bool getLeaderElectionFlg();
            void setLeaderString(std::string leaderSt);
            void setIntersectionStr(std::string intersectionSt);
            void setSolution(std::string solData);
            void setIntersectionflg(bool interFlg);
            void setScheduledFlg(bool schlFlg);
            void setLeaderElectionFlg(bool leadrFlg);
            void setPositionInQueue(int posInqueue);
            void setTurn(int turn);
            int getTurn( );
            void setLight(double lights);
            double getLight();








            //VehicleData update method
            VehicleData update(VehicleData up);
    };



} // namespace veins
