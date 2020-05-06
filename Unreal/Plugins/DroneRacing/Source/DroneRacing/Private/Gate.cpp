#include "Gate.h"
#include "Race.h"

void AGate::registerGatePassThroughHandler(URace *race)
{
	gatePassHandler_ = race;
}


void AGate::BroadcastPassThrough(int gate_id, const AActor *passer)
{
	if (gatePassHandler_ == nullptr)
		return;
	gatePassHandler_->onGatePassedThrough(gate_id, passer);
}