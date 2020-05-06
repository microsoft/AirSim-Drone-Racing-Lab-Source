#include "RacerProgress.h"
#include "Race.h"

const bool RacerProgress::isDisqualified() const
{
	return disqualified_;
}
const bool RacerProgress::isFinished() const
{
	return finished_;
}

const std::set<GateID>& RacerProgress::gatesPassed() const
{
	return gates_passed_;
}

const std::set<GateID>& RacerProgress::gatesSkipped() const
{
	return gates_skipped_;
}

const std::set<FCollisionPenalty> RacerProgress::collisionPenalties() const
{
	return collision_penalties_incurred_;
}

const Time RacerProgress::raceFinishTime() const
{
	return race_finish_time_;
}

const Time RacerProgress::penaltyTime() const
{
	Time penalty_time{ 0 };
	for (auto penalty : collision_penalties_incurred_)
		penalty_time += Time(penalty.severity_multiplier * 1000);
	return penalty_time;
}

void RacerProgress::PassThroughGate(GateID passed_gate, const URace *race)
{
	if (gates_passed_.find(passed_gate) != gates_passed_.end() || gates_skipped_.find(passed_gate) != gates_skipped_.end())
		return;
	for (GateID skipped_gate = race->GateSuccessor(last_gate_passed_); skipped_gate != passed_gate; skipped_gate = race->GateSuccessor(skipped_gate))
	{
		MarkSkippedGate(skipped_gate);
	}
	gates_passed_.insert(passed_gate);
	last_gate_passed_ = passed_gate;

	if (race->IsLastGate(passed_gate))
		MarkFinishedRace();
}

void RacerProgress::AddCollisionPenalty(FCollisionPenalty penalty)
{
	collision_penalties_incurred_.insert(penalty);
}

void RacerProgress::MarkSkippedGate(GateID skipped_gate)
{
	gates_skipped_.insert(skipped_gate);
}

void RacerProgress::MarkFinishedRace()
{
	finished_ = true;
	race_finish_time_ = URace::TimeStamp();
}

const GateID RacerProgress::GetLastGate() const
{
	return last_gate_passed_;
}

void RacerProgress::Disqualify()
{
	disqualified_ = true;
	finished_ = true;
}

void RacerProgress::clear()
{
	gates_passed_.clear();
	gates_skipped_.clear();
	disqualified_ = false;
	finished_ = false;
	race_finish_time_ = INFINITY;
	last_gate_passed_ = NO_GATE;
	collision_penalties_incurred_.clear();
	penalty_time_ = 0;
}