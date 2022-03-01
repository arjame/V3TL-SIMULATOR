//
// Author            : Marco Rapelli
// Date              : December 2019
// Purpose           : Create a BSM message
//

cplusplus {{
#include "veins/base/utils/Coord.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/base/utils/SimpleAddress.h"
}}

namespace veins;

class BaseFrame1609_4;
class noncobject Coord;
class LAddress::L2Type extends void;

packet BasicSafetyMessage extends BaseFrame1609_4 {
	unsigned int msgID;
	unsigned int msgCount;
	unsigned int msgPriority;
	
	unsigned int senderID;
	simtime_t sendingTime;
	Coord senderPosition;
	Coord senderVelocity;
	double senderDirection;
	string senderSignal;
	double senderLength;
	double senderWidth;
	double senderDistance;
	string senderLeaderString;
	string senderIntersectionString;
	string senderSolutionDataset;
	bool senderIntersectionFlag;
	bool senderScheduledFlag;
	bool senderLeaderElectionFlag;
	bool senderPassedIntersectionFlag;
	string senderRoadId;
	int senderPositionInQueue;
	int senderTurn;

}
