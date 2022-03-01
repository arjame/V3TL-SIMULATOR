/*
* Author            : Marco Rapelli & Ahmadreza Jame
* Date              : 2019-2020
* Purpose           : Create a car storage data structure
* Description       : The car storage is a map type composed as <int,VehicleData> where VehicleData is the vehicle instance
*                     and int is the position in the map. DO NOT confuse the position with the vehicle ID: the map is not ordered by ID!!!
*/
#include "veins/modules/application/traci/MyVeinsApp.h"
#include "veins/modules/application/data_structure/CarStorage.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/veins.h"
#include <math.h>
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include <string>
#include<iostream>
#include<fstream>
#include <time.h> /* time */
#include <chrono>
using namespace veins;

bool sortbysec(const pair<int,double> &a, const pair<int,double> &b)
  {
      return (a.second < b.second);
  }
bool sortbysecdec(const pair<int,int> &a, const pair<int,int> &b)
  {
      return (a.second > b.second);
  }

int V3TL_Counter=0;
int FullLeaderCounter=0;
bool stopFlag;
int solutionCounter=0;
double maxDistance,minDistance;
int possibleSolutions[49][5];
int Nc=0;

/**
 * Get the full storage
 */
map<int,VehicleData> CarStorage::getStorage() {
    return storage;
}

/**
 * Get the VehicleData instance of a vehicle from the storage
 */
VehicleData CarStorage::getStorageById(int id) {
    bool found=false;
    map<int,VehicleData>::iterator it;

    for(it=storage.begin(); it!=storage.end(); it++)
        if(it->second.getId()==id)
        {
            found=true;
            break;
        }

    if(found==true)
        return it->second;
    else
        return VehicleData();
}

/**
 * If the given VehicleData is not present in the storage, then it is inserted in a void instances in order to save memory (pooling).
 * If the given VehicleData is present in the storage, it update the corresponding VehicleData occurrence.
 * Eventually it deletes the old entries in the storage.
 */
void CarStorage::updateStorage(VehicleData veh,double oldThreshold) {
    VehicleData up=getStorageById(veh.getId());
    int pooling=0;

    //If vehicle is not present in the storage
    if(up.getId() == VehicleData().getId())
    {
        //Search in the storage for a void VehicleData instance
        for(map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
            //If a void VehicleData instance is found
            if(it->second.getId()==VehicleData().getId())
            {
                //Set the pooling flag
                pooling=1;
                //Put the new vehicle in the position of the void VehicleData instance
                storage[it->first]=up.update(veh);
                break;
            }

        //If a void VehicleData instance is not found
        if(pooling==0)
            //Insert the new vehicle in the end of the storage
            storage[storage.size()]=veh;
    }
    //If vehicle is present in the storage
    else
    {
        //Search for its position in the storage
        for(map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
            if(it->second.getId()==veh.getId())
                //Update the corresponding position
                storage[it->first]=up.update(veh);
    }

    //Delete old entries of the storage
    refreshStorage(oldThreshold);
}

/**
 * Delete entries in storage which are elder than OLD_THRESHOLD
 */
void CarStorage::refreshStorage(double oldThreshold) {
    for(map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
        if(it->second.getId() != VehicleData().getId())
            if(SIMTIME_DBL(simTime())-SIMTIME_DBL(it->second.getTimestamp())>oldThreshold)
                storage[it->first]=VehicleData();

    return;
}

/**
 * Print the storage of a vehicle in the folder v2v_base/outputs
 */
void CarStorage::printStorageById(int id) {

    std::string filename="../../outputs/StorageOf" + std::to_string(id) + ".txt";
    FILE* f_out=fopen(filename.c_str(),"a");

    std::string prefix="######      Storage at time " + std::to_string(SIMTIME_DBL(simTime())) + "      ######\n";
    fputs(prefix.c_str(),f_out);

    for(std::map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)

        {
            std::string vehInfo="\nVehicle #: " + std::to_string(it->second.getId())
                                + " - Timestamp: " + std::to_string(SIMTIME_DBL(it->second.getTimestamp()))
//                                + " - Position: (" + std::to_string(it->second.getPos().x)
//                                + ";" + std::to_string(it->second.getPos().y)
                                + ") - Direction: " + std::to_string(it->second.getDirect())
                                + "scheduled? "+std::to_string(it->second.getScheduledFlg())
                                + ") distance: " + std::to_string(it->second.getDistance())
                                + ") leader: " + std::to_string(it->second.getLeaderElectionFlg())
                                + ") Qposition: " + std::to_string(it->second.getPositionInQueue());




            fputs(vehInfo.c_str(),f_out);
        }

    std::string suffix="\n\n################################################\n\n\n";
    fputs(suffix.c_str(),f_out);

    fclose(f_out);
}
/*
 * function for electing leader according to distances of the cars with the intersection and Nc (maximum number of cars in each group)
 */

int CarStorage::leaderElection(int id,double direction, double distance,int Nc,int Qposition)

   {bool flag;

    //it is for constant leaders
//        for(std::map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
//            if(it->second.getIntersectionflg()==true)
//            {
//                return false;
//                break;
//            }
    int i=0;
 if(Qposition==1)
    {

      for(std::map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
        {   //if car can see another leader in the intersection

            if((it->second.getDirect()!=direction && it->second.getLeaderElectionFlg()==true))
            {   //std::cout<<id<<"see the leader"<<it->second.getId()<<endl;
                flag=true;
            }
            else
            {
               if(it->second.getIntersectionflg()==false)
                if(it->second.getDistance()>distance)
                    if(it->second.getDirect()==direction)
                        i++;
            }
    }


//if there are at least Nc cars behind the car with position 1
if(i>=Nc)
    flag=true;
    }
 for(std::map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
                   if(it->second.getDirect()==direction)
                       if(it->second.getLeaderElectionFlg()==true)
                         if(distance>it->second.getDistance())
                               flag=false;

    return flag;
    }
//    bool value=false;
//    //vector to store instances
//    vector< pair <int,double> > vect;
//    for(std::map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
//        if(it->second.getIntersectionflg()==true)
//        {
//            return false;
//            break;
//        }
//
//    for(std::map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
//    {//if cars are in the same direction
//        if(it->second.getDirect()==direction)
//        {//if car does not pass the intersection
//            if(it->second.getIntersectionflg()==false)
//
//                {//put the instance in the vector
//                //this condition was added because new cars at the beginning of their entrance have a distance=0 which is incorrect to add them in the vector
//                   if(it->second.getDistance()>0.1)
//                    {
//                        vect.push_back( make_pair(it->second.getId(),it->second.getDistance()) );
//                        i++;
//                    }
//
//
//
//                }
//        }
//    }
//
////sort the vector by distance
//sort(vect.begin(), vect.end(), sortbysec);
//
//
//   for(int n=0;n<=i-Nc;n+=Nc)
//     {//checks if we have Nc cars after the leader and the called id is the same as leader
//         if((vect[n].first)==id && n+Nc<=i)
//         {
//             value=true;
//             break;
//         }
//         else
//         value=false;
//
//     }
//
//
//return value;
//




// make leader string with leader's following cars
std::string CarStorage::leaderString(int id,double direction,double distance, int Nc) {
    int i=0;
    int leaderdist=10000;
    std::string leaderString="";
    //vector to store the instances
    vector< pair <double,std::string> > vect;
    //if there are other leaders in front of the car in the direction we don't collect leader string

    //file so save data
//    std::string filename="../../LeaderStrings/leaderStringOf" + std::to_string(id) + ".txt";
//    FILE* f_out=fopen(filename.c_str(),"a");

//    std::string prefix="######      Storage at time " + std::to_string(SIMTIME_DBL(simTime())) + "      ######\n";
//    fputs(prefix.c_str(),f_out);

     for(std::map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
        {
        if(it->second.getDirect()==direction)
            if(it->second.getIntersectionflg()==false)
            {   //it is written to store the cars only behind the specific leader
                double dist=it->second.getDistance()-(distance);
                std::string vehInfo="\nVehicle #:" + std::to_string(it->second.getId())
                                + "- Timestamp: " + std::to_string(SIMTIME_DBL(it->second.getTimestamp()))
                                + " - Position: (" + std::to_string(it->second.getPos().x)
                                + ";" + std::to_string(it->second.getPos().y)
                                + ") - Direction: * "+getDirection(std::to_string(it->second.getDirect()))
                                + ") - Blinking Light &" + (getHeading(it->second.getLights()))
                                + "leader?" + std::to_string(it->second.getLeaderElectionFlg())
                                + " distance :" + std::to_string(it->second.getDistance());

                //if car is behind leader
                if(dist>=0)
                { if(it->second.getPositionInQueue()==1 && it->second.getId()!=id)
                        if(dist<leaderdist)
                            leaderdist=dist;
                   //put the instance in the vector
                    vect.push_back( make_pair(dist,vehInfo) );
                                       i++;
                                       //std::cout<<"incoming id is   :"<<it->second.getDistance()<<" and i    "<<i<<endl;
                                       vect.resize(i);
                }


        }
        }
    //sort cars behind the leader by their distance with respect to leader
    sort(vect.begin(), vect.end());
    //if there are more cars behind leader than Nc put them in leader string
   // if(i>=Nc)
        for(int n=0;n<i;n++)
        {   if(n<Nc && vect[n].first<leaderdist)
            {   leaderString+=(vect[n].second);
            //replace the position with instantaneous queue position
                leaderString+="   Qposition:>"+std::to_string(n+1);
            }


        }
//    fputs(leaderString.c_str(),f_out);

//    std::string suffix="\n\n################################################\n\n\n";
//    fputs(suffix.c_str(),f_out);
//    fclose(f_out);

return leaderString;
}
// cheking if the vehicle passes the intersection
bool CarStorage::isPassedIntersection(int id,/*double distance)*/std::string Roadid)
{

//   for(map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
//   { if(it->second.getId()==id)
//    // {if((it->second.getDistance()<1000) && (distance<1000))
//    // {  if((it->second.getDistance()>0) && (distance>0))
//         { if(it->second.getDistance()>=distance)
//
//
//             {//std::cout<<it->second.getDistance()<<"id is "<<it->second.getId()<<"input distance is"<<distance<<"\n\n************\n\n";
//             return false;
//             }
//             else
//             {//std::cout<<it->second.getDistance()<<"id is "<<it->second.getId()<<"input distance is"<<distance<<"\n\n************\n\n";
//                 return true;
//             }
//         }
//     }



   //  }
 //  }

 /*this part is working with with roadId
  *
  */
    for(map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
        if(it->second.getId()==id)
        {
            if(it->second.getRoadId()!=Roadid)
                   {
                       return true;

                     break;
                   }
        }
    return false;
//return false;
}
/*
 * function to build intersection string by collecting leaders information
 */
std::string CarStorage::intersectionString(int id,double direction,double distance) {
    int i=0;
  //  std::string filename="../../IntersectionStrings/InterStrOf" + std::to_string(id) + ".txt";

   // FILE* f_out=fopen(filename.c_str(),"a");
        std::string leader;
//        std::string prefix="######      Storage at time " + std::to_string(SIMTIME_DBL(simTime())) + "      ######\n";
//       fputs(prefix.c_str(),f_out);
        std::string intersectionString="";
            for(std::map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
                        {
                          if(it->second.getLeaderElectionFlg()==true && it->second.getLeader()!="")
                              //it was added to consider just nearest leaders as intersection string
                              if(it->second.getDistance()<100)
                              {
                                  leader=it->second.getLeader();
                                  intersectionString+=leader;
                                  i++;
                              }

                        }

//if(i>1)
//{
//    resultWriter(intersectionString,i,id);
//}
//if(i==4)
//{ //auto t1 = std::chrono::high_resolution_clock::now();
//solutionUpdater(intersectionString);
//auto t2 = std::chrono::high_resolution_clock::now();

//auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();

//std::cout <<"duration is "<< duration<<endl;
//}

//   fputs(intersectionString.c_str(),f_out);
//   std::string suffix="\n\n################################################\n\n\n";
//   fputs(suffix.c_str(),f_out);
//   fclose(f_out);

   return intersectionString;

}
// this function returns the id of nearest car ahead of the vehicle, if there is no one in front of the car, it returns -1
int CarStorage::nearestCarAhead(int id, double distance,double direction)

{
    double diference=100000;
    int nearestid=-2;
    int i=0;
    for(map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
        {
                if(it->second.getDirect()==direction)
                    if(it->second.getIntersectionflg()==false)
                        //if((it->second.getDistance()>0.1) && (distance>0.1))
                    {
                        double dist=(distance)-it->second.getDistance();
                        if(dist<diference && dist>0)
                            {
                                i++;
                                diference=dist;
                                nearestid=it->second.getId();

                            }

                    }

        }
   if(i<1)
      nearestid=-1;


return nearestid;

}
//this function returns the nearest car position in order to set the queue position
int CarStorage::QpositionSeter(int id, double distance,double direction, bool scheduleFlag)

    {
    bool flag=false;
    int i=-2;
//    if(scheduleFlag==true)
//        return -2;
    //freeze changing the positions
    //if(distance<100)
     //   return -2;
    if(distance<990)
       // std::cout<<distance<<"<--idistance i-->"<<i<<endl;
//it is written for vehicle if it can not see the leader it should not join to that group
        for(std::map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
            if(it->second.getPositionInQueue()==1 && distance>it->second.getDistance())
                flag=true;




    if (flag==true)
    {
        i=this->nearestCarAhead(id, distance,direction);

    for(std::map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
        {
        //if the car can see the position 1 it means it can join to that group
            if(it->second.getId()==i)
            {
                if(it->second.getScheduledFlg()==false)
                {
                    i=it->second.getPositionInQueue();
                    return (i);
                //break;
                }

            }
        }
   // std::cout<<id<<"<--id position-->"<<i<<endl;
}
    else
        return -1;

return i;

}
/*
 * writing the results of simulation
 */
void CarStorage::resultWriter(std::string Str, int numLeaders, int id)

{
    maxDistance=0;
    minDistance=10000;

    //it counts number of cars in each intersection string
    int count=0;
    //collects distances
    std::string distStr;

    //it is global variable to count how many times function was called each call means we have V3TL triggered
    V3TL_Counter++;
        //for counting number of cars
    std::string line;
    std::istringstream string(Str);
    while(std::getline(string, line)) {

    for (int n = 0; n < line.size(); n++)
        {    if (line.at(n) == '#')
                      count++;
            if(line.at(n)=='?')
                //to collect distances of leaders
            {
                if(line.at(n+1)=='1')
                    {

                        for(int l=13;l<22;l++)
                        distStr+=line.at(n+l);
                         double distance=stoi(distStr);
                         if(distance<minDistance)
                             minDistance=distance;
                         else if(distance>maxDistance)
                             maxDistance=distance;
                        distStr+="  ";
                    }
            }

        }
   // std::cout<<heading<<" "<<pos<<" "<<dir<<endl;


    }


std::string name="../../Results/V3TLforpy.txt";
FILE* file_out=fopen(name.c_str(),"a");
std::string mE=std::to_string(SIMTIME_DBL(simTime()))
+"\t"+std::to_string(numLeaders)
+"\t"+std::to_string(count)
+"\t"+std::to_string(minDistance)
+"\t"+std::to_string(maxDistance)+"\n";
fputs(mE.c_str(),file_out);
fclose(file_out);

        std::string filename="../../Results/V3TL.txt";
        FILE* f_out=fopen(filename.c_str(),"a");
        std::string prefix="######      V3TL at time " + std::to_string(SIMTIME_DBL(simTime())) + "      ######\n";
        fputs(prefix.c_str(),f_out);
        std::string RE="\nVehicle #: " + std::to_string(id)
        + "(Number of leaders it can see: " + std::to_string(numLeaders)
        + ")(Total times that V3TL triggered:" + std::to_string(V3TL_Counter)
        + "))Number of cars Involved: " + std::to_string(count)
        + ")(Leader distances: " + distStr+")";

        fputs(RE.c_str(),f_out);

        std::string suffix="\n\n################################################\n\n\n";
           fputs(suffix.c_str(),f_out);
           fclose(f_out);
if(numLeaders==4)
{
    //counts how many times we have full intersection map
FullLeaderCounter++;


    std::string filename="../../Results/FullIntersection.txt";
    FILE* f_out=fopen(filename.c_str(),"a");
    std::string prefix="######   4 way full V3TL at time " + std::to_string(SIMTIME_DBL(simTime())) + "      ######\n";
    fputs(prefix.c_str(),f_out);
    std::string RE="\nVehicle #:" + std::to_string(id)+
    + "(Number of leaders it can see: " + std::to_string(numLeaders)
    + ")(Total times that V3TL triggered:" + std::to_string(FullLeaderCounter)
    + ")(Number of cars Involved:" + std::to_string(count)
    + ")(Leader distances: " + distStr+")";
    fputs(RE.c_str(),f_out);

std::string suffix="\n\n################################################\n\n\n";
fputs(suffix.c_str(),f_out);
fclose(f_out);

}

}

std::string CarStorage::getHeading(std::string route)
{
    //going straight
if(route=="route1" || route=="route2" || route=="route3" || route=="route4")
    return "3";
//turning right
else if(route=="route5" || route=="route7" || route=="route10" || route=="route11")
    return "1";
//turning left
else if(route=="route6" || route=="route8" || route=="route9" || route=="route12")
    return "2";
else
    return "error";

}

std::string CarStorage::getDirection(std::string direction)
{   //north
    if(direction=="-1.570796")
        return "0";
    //south
    if(direction=="1.570796")
            return "1";
    //west
    if(direction=="0.000000")
            return "2";
    //east
    if(direction=="-3.141593")
                return "3";
    else
        return "error";


}
void CarStorage::legalMoves()
{

    std::string name="../../possible_moves.txt";
    int i=0;
    //reading the full solution file to extract the legal movements
    std::string line1;
    std::ifstream file(name.c_str());
    while(getline(file, line1)) {
        for (int n = 0; n < line1.size(); n++)
            possibleSolutions[i][n]=(line1.at(n))-'0';
            i++;
    }

    file.close();

}
/*
 * direction priority is north=0 south=1 west=2 east=3
 */
 std::vector<int> CarStorage::combinationsCreator(int a[4])

//maximum 16 combinations for each full 4 cars 4moves, 3 moves one stop, etc,.

{
    //this is for understanding how many cars are in each direction in line



//    for (int j=0;j<4;j++)
//       std::cout<<noInPosition[j]<<"  "<<endl;

    int combination[15][4];

for(int i=0;i<15;i++)
    {
       for(int j=0;j<4;j++)
       {//4 moves
        if(i==0)
            combination[i][j]=a[j];
        //3 moves
        else if(i<5)
            if(i-1==j)
            combination[i][j]=0;
            else
                combination[i][j]=a[j];
        //two moves
        else if(i<8)
            if(i-5==j || i-4==j)
                combination[i][j]=0;
            else
                combination[i][j]=a[j];
        else if(i<10)
            if(i-8==j || i-6==j)
                combination[i][j]=0;
            else
                combination[i][j]=a[j];
         else if(i==10)
                if(j==0 || j==3)
                    combination[i][j]=0;
                else
                    combination[i][j]=a[j];
        //one moves
         else
             if(i-11==j)
                 combination[i][j]=a[j];
             else
                 combination[i][j]=0;

       }


    }
//std::cout<<"we are here"<<endl;
vector< pair <int,int> > vect;
//vector< pair <int,int> > stopAndGo;
vector<int> result;



//int possibleSolutions[49][5];
//std::string name="/Users/ahmadreza/Desktop/V3Tl_results/mm.txt";
//int i=0;
////reading the full solution file to extract the legal movements
//std::string line;
//std::ifstream file(name.c_str());
//while(getline(file, line)) {
//    for (int n = 0; n < line.size(); n++)
//        possibleSolutions[i][n]=(line.at(n))-'0';
//        i++;
//}
//comparing combinatios of 4 cars with legal moves to get the matches, matches are safe movements of 4 cars
int o=0;
int h=0;
for(int i=0; i<49; i++)
{
    for(int j=0; j<15; j++)
    {
        o=0;
        for(int k=0;k<4;k++)
        {
            if(possibleSolutions[i][k]==combination[j][k])
                o++;
        }
   if(o==4)
       {
       vect.push_back(make_pair(i,possibleSolutions[i][4]));
    //std::cout<<"solution"<<vect[h].first<<"no: "<<vect[h].second<<endl;
       h++;
       }
    }

}
//std::cout<<"h is"<<h<<endl;
sort(vect.begin(), vect.end(),sortbysecdec);
result.push_back(vect[0].first);
if(h>1)
{
    for(int i=1;i<h;i++)
    {
        if(vect[i].second>=vect[0].second && vect[i].first!=vect[0].first)
        {   result.push_back(vect[i].first);
            //std::cout<<result[i]<<endl;
        }
    }
}

//int index=0;
////sort vector decending
//sort(vect.begin(), vect.end(),sortbysecdec);
//
////calculate number of stop n go of cars if we choose this solution
//stopAndGo.push_back(make_pair(vect[0].first,stopNgoCounter(vect[0].first,noInPosition)));
//for(int i=1;i<h;i++)
//{
//    if(vect[i].second>=vect[0].second)
//        {
//        //stopAndGo.resize(index);
//        stopAndGo.push_back(make_pair(vect[i].first,stopNgoCounter(vect[i].first,noInPosition)));
//        index++;
//        }
//
//}
//if(index>0)
//{   int p=0;
//    sort(stopAndGo.begin(), stopAndGo.end(),sortbysec);
//    for(int i=1;i<=index;i++)
//        if(stopAndGo[i].second<=stopAndGo[0].second)
//            p++;
//    if(p>0)
//    {
//        srand(time(NULL));
//        int i=rand()%(p+1);
//        final_solution=stopAndGo[i].first;
//     // std::cout<<"after randomize the solution is "<<i<<p<<endl;
//    }
//    else
//        final_solution=stopAndGo[0].first;
//
//}
//else
//    final_solution=stopAndGo[0].first;
//

//std::cout<<"final solution is this"<<final_solution<<endl;

//std::cout<<"\n\n\n";


return result;


}




int CarStorage::stopNgoCounter(int i,int noInPosition[4])

{
   int counter=0;
   for(int j=0;j<4;j++)
       if(possibleSolutions[i][j]!=0)
           counter+=noInPosition[j];
return counter;
}


std::string CarStorage::solutionFinder(std::string Str,int id, int Nc,double direction)

{
//if one of the leaders have already calculated the solution,other leaders just use it
//    int leaderCounter=1;
//for(std::map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
//       if(it->second.getDirect()!=direction && it->second.getLeaderElectionFlg()==true && it->second.getDistance()<100)
//           leaderCounter++;
////if(leaderCounter>3)
////{
////    std::cout<<leaderCounter<<endl;
////    std::cout<<simTime()<<endl;
////}
//if(leaderCounter==4)
//{
 //this line is for calculating the duration of the function
 std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//maximum size of fullsolution is Nc*4
int FullSolution[Nc*4][4];
int noInPosition[4];
int numberOfCars=0;
int carId;
//call the function
//receive answer updade for nex call
//call again
int map[Nc][4];
int update[4];
int list[4];
int mapIds[Nc][4];
int dir,pos,heading,leader,i=0;
dir=pos=heading=leader=0;


for(int i=0;i<Nc;i++)
    for(int j=0;j<4;j++)
    {  map[i][j]=0;
      mapIds[i][j]=-1;
    }
for(int j=0;j<4;j++)
{
    update[j]=0;
    noInPosition[j]=0;
    list[j]=0;
}


std::string line;
std::istringstream string(Str);

  while(std::getline(string, line))
  {
      for (int n = 0; n < line.size(); n++)
      {
               if(line.at(n)=='#')
                  { i=2;
                   std::string carIdStr;
                      while(line.at(n+i)!='-')
                       {
                         carIdStr+=to_string(line.at(n+i)-'0');
                         i++;
                        }
                         carId=std::stoi(carIdStr);
                  }
          //after * we have direction
          if(line.at(n)=='*')
              //convert char to int
           dir=line.at(n+2)-'0';


          //this is for position of car
              if(line.at(n)=='>')
                  //to convert char to int -48, and -1 because position starts from 1 but matrix starts from 0
               pos=line.at(n+1)-49;

              if(line.at(n)=='&')
              heading=line.at(n+1)-'0';
             if(line.at(n)=='?')
                  if(line.at(n+1)=='1')
                      leader++;

      }
             map[pos][dir]=heading;
             mapIds[pos][dir]=carId;
}
//  for (int i=0;i<6;i++)
//  {  for (int j=0;j<4;j++)
//          std::cout<<map[i][j]<<"  ";
//  std::cout<<endl;
//  }
//  std::cout<<"\n\n\n";
  for (int i=0;i<4;i++)
    {
      int count=0;
      for(int j=0; j<Nc; j++)
            if(map[j][i]!=0)
                count++;
      noInPosition[i]=count;
      numberOfCars+=noInPosition[i];

  // std::cout<<"  noInPosition "<<noInPosition[i]<<endl;
    }

if(leader>2)
    //std::cout<<simTime()<<endl;
{
    std::string fileStr=to_string(numberOfCars)+"\t";



    //if stop sets to true means involdev cars in the solution should stop at the edge of the street
    stopFlag=true;


    //counts the number of levels with clear the junction
  int counter=0;
//while(numberOfCars>0)
  //{   //std::cout<<" numberOfCars:  "<<numberOfCars<<endl;

    //update[i] stores the position of each column
//    for (int i=0;i<4;i++)
//    {
//        if(update[i]<Nc)
//            list[i]=map[update[i]][i];
//        else
//        { list[i]=0;
//        }
//
//    }


      for (int i=0;i<4;i++)
        list[i]=map[0][i];

 std::vector<int> sol1;
 std::vector<int> sol2;
 std::vector<pair<int,int>> StopNgocounter;



//here main function calls another function and send the tier for second function in order to get back the solution for that tier
 while(numberOfCars>0)
 {     //std::cout<<"1"<<endl;


     int final_solution=0;
        sol1=combinationsCreator(list);
     int*p1,*p2;
  if(sol1.size()>1)
  {//std::cout<<"2"<<endl;
      vector< pair <int,int> > cars;
       for(int i=0;i<sol1.size();i++)
       {  // std::cout<<"3"<<endl;
           int tempupdate[4],tempnoInPosition[4];
           for(int o=0;o<4;o++)
           {    //std::cout<<"4"<<endl;
               tempupdate[o]=update[o];
               tempnoInPosition[o]=noInPosition[o];
           }
          p1=updateSolution(Nc,map,tempupdate,sol1[i]);
          for(int j=0;j<4;j++)
              {
                  tempupdate[j]=*(p1+j);
                  if(tempupdate[j]<Nc)
                      list[j]=map[tempupdate[j]][j];
                  else
                      list[j]=0;
                 // std::cout<<"list[j]"<<list[j]<<endl;
                  cars.push_back(make_pair(sol1[i],possibleSolutions[sol1[i]][4]));
                  //std::cout<<"6"<<endl;
                  if(possibleSolutions[sol1[i]][j]!=0)
                      tempnoInPosition[j]--;
                 // std::cout<<"tempnoInPosition[j]--"<<tempnoInPosition[j]<<endl;
              }
          StopNgocounter.push_back(make_pair(sol1[i],stopNgoCounter(sol1[i],tempnoInPosition)));
          //std::cout<<"8"<<endl;
          //std::cout<<"cars"<<cars[i].second<<endl;
          sol2=combinationsCreator(list);
          cars[i].second+=possibleSolutions[sol2[0]][4];
          //std::cout<<"cars"<<cars[i].second<<endl;
          //std::cout<<"9"<<endl;
       }
       sort(cars.begin(),cars.end(),sortbysecdec);
       vector< pair <int,int> > stopAndGo;
       for(int k=0;k<cars.size();k++)
       {//std::cout<<"10"<<endl;
           if(cars[k].second>=cars[0].second)
           {//std::cout<<"11"<<endl;
               for(int j=0;j<StopNgocounter.size();j++)
                   if(cars[k].first==StopNgocounter[j].first)
                       stopAndGo.push_back(make_pair(StopNgocounter[j].first,StopNgocounter[j].second));
           }
           //std::cout<<"12"<<endl;
       }

       sort(stopAndGo.begin(),stopAndGo.end(),sortbysec);
       //std::cout<<"13"<<endl;
       if(stopAndGo.size()>1)
       {  //std::cout<<"14"<<endl;
           int w=0;
           for(int l=0;l<stopAndGo.size();l++)
           {//std::cout<<"15"<<endl;
               if(stopAndGo[l].second<=stopAndGo[0].second)
                   w++;
               //std::cout<<"stopAndGo[l].second"<<endl;
           }
           srand(time(NULL));
           int i=rand()%(w);
           final_solution=stopAndGo[i].first;
           numberOfCars-=possibleSolutions[final_solution][4];
           //std::cout<<"final_solution random"<<final_solution<<endl;

       }
       else
           {//std::cout<<"17"<<endl;
               final_solution=cars[0].first;
               numberOfCars-=possibleSolutions[final_solution][4];
           }
       p2=updateSolution(Nc,map,update,final_solution);
       //std::cout<<"18"<<endl;
       for(int z=0;z<4;z++)
            {
                update[z]=*(p2+z);
                if(update[z]<Nc)
                list[z]=map[update[z]][z];
                else
                    list[z]=0;
                if(possibleSolutions[final_solution][z]!=0)
                noInPosition[z]--;
                //std::cout<<"noInPosition[z]--"<<noInPosition[z]<<endl;

            }

  }
  else
  {//std::cout<<"20"<<endl;
      final_solution=sol1[0];
      p1=updateSolution(Nc,map,update,final_solution);
      //std::cout<<"21"<<endl;
      for(int i=0;i<4;i++)
      {
          update[i]=*(p1+i);
if(update[i]<Nc)
          list[i]=map[update[i]][i];
else
    list[i]=0;
if(possibleSolutions[final_solution][i]!=0)
                noInPosition[i]--;
//std::cout<<"update i"<<update[i]<<endl;
      }
      numberOfCars-=possibleSolutions[final_solution][4];
      //std::cout<<"number of cars"<<numberOfCars<<endl;
  }
  for (int i=0;i<4;i++)
          {
              //move[i]=possibles[solution][i];
              if(possibleSolutions[final_solution][i]!=0)
              {
                  if(update[i]>0)
                      FullSolution[counter][i]=mapIds[update[i]-1][i];

              else
                 FullSolution[counter][i]=-1;

              }
              else
                  FullSolution[counter][i]=-1;
          }

  counter++;
 }
      //update total number of remaining cars to be scheduled
     //   numberOfCars-=possibleSolutions[sol[0]][4];
//this is for updating the pointers to each column and get the next tier to be shceduled


//making solution string
std::string solution="";
for(int i=0; i<counter; i++)
{
    //solution+="#";
    for(int j=0;j<4;j++)
    {

    solution+=std::to_string((FullSolution[i][j]));
    solution+="\n";

    }
}
//  for (int i=0;i<counter;i++)
//  {  for (int j=0;j<4;j++)
//          std::cout<<FullSolution[i][j]<<"  ";
//  std::cout<<endl;
//  }
////std::cout<<"total times that solution triggered:"<<solutionCounter<<endl;
//std::cout<<"\n\n\n";
//std::cout<<solution<<"\n\n\n";

//count the total time that solution provided
solutionCounter++;

//records duration of running the function
std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

//calculate the duration
double d=std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();

//recording statistics in the files
fileStr+=to_string(d)+"\t"+to_string(solutionCounter)+"\t"+to_string(counter)+"\n";
std::string name="../../Results/V3TLALGforpy.txt";
FILE* f_out=fopen(name.c_str(),"a");
fputs(fileStr.c_str(),f_out);
fclose(f_out);

return solution;

 }
  else
      return "";
}



std::string CarStorage::solutionSetter(int id,double direction,double distance)
{
//vector< pair <double,std::string> > vect;
std::string solution;
int carId;
    for(map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
        if(it->second.getSolution()!="")
        {
            solution=it->second.getSolution();
            std::string line;
            std::istringstream string(solution);
            int i=0;
            while(std::getline(string, line))
            {
                carId=std::stoi(line);


                if(id==carId)
                  {
                    //std::cout<<"\n\n id "<<id<<endl;
                    //std::cout<<"trun  "<<i/4<<" id "<<carId<<endl;

                      return solution;


                  }
                i++;
            }
                //int i;
//                for (int n = 0; n < line.size(); n++)
//                {
//                    if(line.at(n)=='#')
//                    {
//                        i=1;
//                        std::string carIdStr;
//                        while(line.at(n+i)!='^')
//                        {
//                            carIdStr+=to_string(line.at(n+i)-'0');
//                            i++;
//                        }
//                carId=std::stoi(carIdStr);
//                //std::cout<<"matched id is "<<carIdStr<<endl;
////                if(id==carId)
////                {
////                    return solution;
////                }
//              }
//              //after * we have direction
//          }
//                std::cout<<"ids "<<carId<<endl;
//
//      }

//int i=0;
//    for(map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)
//       if(it->second.getPositionInQueue()==1)
//           if(it->second.getDirect()==direction)
//               if(it->second.getId()!=id)
//                   if(it->second.getScheduledFlg()==true)
//                       if(it->second.getDistance()<distance)
//                           //if(it->second.getSolution()!="")
//                   {  int dist=distance-it->second.getDistance();
//                           vect.push_back(make_pair(dist,it->second.getSolution()));
//                   i++;
//                   }
//
//sort(vect.begin(), vect.end());
//if(i>0)
//{
//    return (vect[0].second);}
//else
//    return "";

}
return "";
}

bool CarStorage::stopCaller()

{
    if(stopFlag==true)

    {   stopFlag=false;
        return true;
    }
else
    return false;


}
int * CarStorage::updateSolution(int Nc,int dataset[Nc][4],int positionsInColumns[4],int solution ) {

    static int updatedPositions[4];

    for(int i=0;i<4;i++)
        {
           if (possibleSolutions[solution][i]!=0)
                updatedPositions[i]=positionsInColumns[i]+1;
           else
               updatedPositions[i]=positionsInColumns[i];

        }


    return updatedPositions;

}

bool CarStorage::moveAllower(int id, int turn, std::string roadId, double distance,bool interfalg)
{
    bool flag=true;
    for(map<int,VehicleData>::iterator it=storage.begin(); it!=storage.end(); it++)

    {   if(it->second.getTurn()==(turn-1))
                if(it->second.getTurn()!=-1)
                   // if(it->second.getDistance()<distance && roadId!=it->second.getRoadId())
                        flag=false;
    if(turn==-1 && it->second.getTurn()>=0)
        flag=false;

    }

//        {//std::cout<<"id caller: "<<id<<" turn second car: "<<it->second.getTurn()<<"id second car:"<<it->second.getId()<<endl;
//              if(it->second.getDistance()<distance)
//                    if(it->second.getIntersectionflg()==false)
//                        {
//                            flag=false;
//                        //std::cout<<id<<" this car "<<it->second.getId()<<endl;
//                        }
//        }
    if(interfalg==true)
        flag=true;
return flag;


}

void CarStorage::recorder(int count,int stopNgo, int shedcount)
{

    std::string name="../../Results/V3TLConfid.txt";
    FILE* f_out=fopen(name.c_str(),"a");
    std::string str=std::to_string(count)+"\t"+std::to_string(SIMTIME_DBL(simTime()))+"\t"+std::to_string(stopNgo)+"\t"+std::to_string(shedcount)+"\n";
    fputs(str.c_str(),f_out);

    fclose(f_out);
}
