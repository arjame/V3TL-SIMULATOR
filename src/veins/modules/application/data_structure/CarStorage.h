/*
* Author            : Marco Rapelli
* Date              : December 2019
* Purpose           : Create a car storage data structure
* Description       : The car storage is a map type composed as <int,VehicleData> where VehicleData is the vehicle instance
*                     and int is the position in the map. DO NOT confuse the position with the vehicle ID: the map is not ordered by ID!!!
*/

#pragma once

#include "veins/veins.h"

#include <map>
#include "veins/modules/application/traci/MyVeinsApp.h"
#include "veins/base/utils/Coord.h"
#include "veins/modules/application/data_structure/VehicleData.h"

using namespace std;
using namespace omnetpp;

namespace veins {
    class CarStorage {
        protected:
            //old threshold
            double oldThreshold;
            //the storage is a list of VehicleData structures
            map<int,VehicleData> storage;

        public:
            //void instance of CarStorage
            CarStorage() {
                storage = std::map<int,VehicleData> ();
            }


            //CarStorage methods
            std::map<int,VehicleData> getStorage();
            VehicleData getStorageById(int id);
             void updateStorage(VehicleData veh,double oldThreshold) ;
            void refreshStorage(double oldThreshold) ;
            std::map<int,VehicleData> deleteEntryById(int id);
            void printStorageById(int id) ;
            std::string leaderString(int id , double direction,double distance, int Nc) ;
            void collectDistancesByid(int id,double distance);
            int leaderElection(int id,double direction,double distance,int Nc,int Qposition);
            bool isPassedIntersection(int id ,/*double distance);// this is for second bunch of code*/ std::string Roadid);
            std::string intersectionString(int id,double direction,double distance);
            int nearestCarAhead(int id, double distance,double direction);
            int QpositionSeter(int id, double distance,double direction,bool scheduleFlag);
            void resultWriter(std::string Str,int numLeaders, int id);
            std::string getHeading(std::string route);
            void legalMoves();
            std::string getDirection(std::string direction);
            std::vector<int> combinationsCreator(int a[4]);
            int stopNgoCounter(int i,int noInPosition[4]);
            std::string solutionFinder(std::string Str, int id, int Nc, double direction);
            std::string solutionSetter(int id,double direction,double distance);
            bool stopCaller();
            int * updateSolution(int Nc,int dataset[Nc][4],int positionsInColumns[4],int solution );
            bool moveAllower(int id, int turn,std::string roadId, double distance,bool interflag);
            void recorder(int count, int stopNgo, int shedcount);


    };

} // namespace veins
