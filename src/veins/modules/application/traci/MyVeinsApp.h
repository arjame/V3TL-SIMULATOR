/*
* Author            : Marco Rapelli
* Date              : December 2019
* Purpose           : Create a baseline for Vehicle-to-Vehicle simulated application
*/

#pragma once

#include "veins/veins.h"
#include <math.h>
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/application/data_structure/CarStorage.h"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

using namespace omnetpp;

namespace veins {

/**
 * @brief
 * This is a stub for a typical Veins application layer.
 * Most common functions are overloaded.
 * See MyVeinsApp.cc for hints
 *
 * @author David Eckhoff
 *
 */

class VEINS_API MyVeinsApp : public DemoBaseApplLayer {
public:
    void initialize(int stage) override;
    void finish() override;


protected:
    //Pointers to the mobility environment
    TraCIMobility* mobility;
    TraCICommandInterface* traci;
    TraCICommandInterface::Vehicle* traciVehicle;
    //Instances of VehicleData and CarStorage
    CarStorage storage;
    VehicleData vehicle;
    //Old threshold
    double oldThreshold;


    int Nc;
    std::string RiD="0";
    //MyVeinsApp methods
    //NOTE: to change the message type if an other type of message is used!!
    void onBSM(BasicSafetyMessage* bsm) override;
    void onWhereAmI() override;
    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;

};

} // namespace veins
