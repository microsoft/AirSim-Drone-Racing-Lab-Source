#pragma once
#include "RacingDefines.h"
#include <set>

class URace;

class RacerProgress
{
public:

    //Getters.
	const bool isDisqualified() const;
	const bool isFinished() const;
	const std::set<GateID>& gatesPassed() const;
	const std::set<GateID>& gatesSkipped() const;
	const std::set<FCollisionPenalty> collisionPenalties() const;
	const Time raceFinishTime() const;
	const Time penaltyTime() const;
	//Update data.
	void PassThroughGate(GateID passed_gate, const URace *race);
	void AddCollisionPenalty(FCollisionPenalty penalty);
	const GateID GetLastGate() const;
	void Disqualify();
	void clear();
private:
	void MarkSkippedGate(GateID skipped_gate);
	void MarkFinishedRace();

private:
	std::set<GateID> gates_passed_;
	std::set<GateID> gates_skipped_;
	bool disqualified_ = false;
	bool finished_ = false;
	Time race_finish_time_ = INFINITY;
	GateID last_gate_passed_ = NO_GATE;
	std::set<FCollisionPenalty> collision_penalties_incurred_;
	bool collided_ = false;

	Time penalty_time_{ 0 };
};