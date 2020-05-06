//#pragma once
#include "Race.h"
#include "Racer.h"
#include "CoreMinimal.h"
#include "Raceable.h"
#include "Gate.h"
#include "RacingLibrary.h"
#include "Raceable.h"

void URace::initialize(UWorld* world_context, const FVector start_block)
{
	start_block_ = start_block;
	world_ = world_context;
	URacingLib::getGates(world_, gates_);
	auto race_actors = URacingLib::getRacers(world_);
	participants_ = convert(race_actors);
	orderGates();
	subscribeToGatesEvents();
	subscribeToCollisionEvents();
}

void URace::start(int tier)
{
	is_racing_ = true;
	for (auto *racer : participants_)
	{
		racer->StartRacing();
		if (tier == 2 && racer->name_ == BASELINE_NAME)
			racer->Disqualify();
	}
}
void URace::end()
{
	is_racing_ = false;
	for (auto *racer : participants_)
		racer->StopRacing();
}

void URace::subscribeToGatesEvents()
{
	for (auto *gate : gates_)
	{
		gate->registerGatePassThroughHandler(this);
	}
}
void URace::subscribeToCollisionEvents()
{
	for (auto racer : participants_)
	{
		racer->collision_event_.AddDynamic(this, &URace::Penalize);
	}
}

bool URace::droneIsFollower(ARacer* racer, ARacer* collider)
{
	// get next gates
	auto racer_last_gate = racer->getProgress().GetLastGate();
	auto collider_last_gate = collider->getProgress().GetLastGate();

	auto temp_distance1 = normDistance(racer->getRacingActor(), racer_last_gate);
	auto temp_distance2 = normDistance(collider->getRacingActor(), collider_last_gate);
	return racer_last_gate < collider_last_gate || 
		normDistance(racer->getRacingActor(), racer_last_gate) < normDistance(collider->getRacingActor(), collider_last_gate);
}

float URace::normDistance(const AActor* drone, const GateID& gate_id)
{
	FVector path_vector;
	FVector drone_vector;
	GateID next_gate = GateSuccessor(gate_id);

	if (next_gate == NO_GATE) //We have already finished the race.
		return 0.f; //The most correct thing to say is that we perpetually have no distance left to cover to pass the next gate.

	if (gate_id != NO_GATE && next_gate != NO_GATE) //Post first gate, pre last gate.
	{
		auto **gate1 = gates_.FindByPredicate([gate_id](AGate* g) {return g->id_ == gate_id;});
		auto **gate2 = gates_.FindByPredicate([next_gate](AGate* g) {return g->id_ == next_gate; });

		path_vector = (*gate2)->GetActorLocation() - (*gate1)->GetActorLocation();
		drone_vector = drone->GetActorLocation() - (*gate1)->GetActorLocation();
	}
	else //Pre first gate case.
	{
		auto** gate = gates_.FindByPredicate([](AGate* g) {return g->id_ == 0;});
		//Distance from a point that's equidistant from starting points.
		path_vector = (*gate)->GetActorLocation() - start_block_;
		drone_vector = drone->GetActorLocation() - start_block_;
	}
	return FVector::DotProduct(path_vector, drone_vector);
}

void URace::Penalize(ARacer* racer, ECollision collision_type, float severity_multiplier, const AActor* actor_collided_with)
{
	if (!is_racing_)
		return;
	switch (collision_type)
	{
	case ECollision::DroneCollision :
		if( droneIsFollower(racer, (ARacer*)actor_collided_with))
			racer->Disqualify();
		break;
	case ECollision::SafetyViolation :
		if (droneIsFollower(racer, (ARacer*)actor_collided_with))
			racer->getProgress().AddCollisionPenalty(FCollisionPenalty(collision_type, TimeStamp(), severity_multiplier, (ARacer*)actor_collided_with));
		break;
	case ECollision::EnvironmentCollision :
		racer->getProgress().AddCollisionPenalty(FCollisionPenalty(collision_type, TimeStamp(), severity_multiplier));
		break;
	}
}

void URace::onGatePassedThrough(int id, const AActor *passer)
{
	if (!is_racing_)
		return;
	for (auto *racer : participants_)
	{
		if (racer->getRacingActor() == passer)
		{
			racer->getProgress().PassThroughGate(static_cast<GateID>(id), this);
			break;
		}
	}
}

TArray<ARacer*> URace::convert(const TArray<AActor*>& race_actors)
{
	//convert to Racer Objects
	TArray<ARacer*> racers;
	for (auto race_actor : race_actors)
	{
		if (race_actor->GetClass()->ImplementsInterface(URaceable::StaticClass()))
		{
			UClass* aracer_class = ARacer::StaticClass();
			ARacer *racer = world_->SpawnActor<ARacer>(aracer_class, FVector(0.0f, 0.f, 0.f), FRotator(0.0f, 0.f, 0.f), FActorSpawnParameters());

			IRaceable::Execute_AssignRacer(race_actor, racer);
			racer->setActor(race_actor);
			racers.Add(racer);
		}
	}
	return racers; //placeholder
}

void URace::orderGates()
{
	if (gates_.Num() == 0)
		return;

	// Convert vector of unordered gate IDs to map of gates and their successors
	gate_successors_.clear();
	gate_successors_.insert(std::pair<GateID, GateID>(NO_GATE, gates_[0]->id_)); //Successor to 'no passed gates' is first gate.
	for (int i =0; i< gates_.Num() -1; i++)
	{
		gate_successors_.insert(std::pair<GateID, GateID>(gates_[i]->id_, gates_[i + 1]->id_));
	}
	final_gate_ = gates_[gates_.Num() - 1]->id_;
	number_of_gates_ = gates_.Num();
}

int URace::NumberOfGates() const
{
	return number_of_gates_;
}

void URace::placeRacersAtStartPositions(const TArray<FTransform>& player_starts, bool is_first_race)
{
	if (participants_.Num() == 2)
	{
		if (is_first_race)
		{
			//participants_[0]->setPose(player_starts[0]->GetActorLocation(), player_starts[0]->GetActorRotation());
			participants_[0]->setPose(player_starts[0].GetLocation(), player_starts[0].GetRotation().Rotator());
			participants_[1]->setPose(player_starts[1].GetLocation(), player_starts[1].GetRotation().Rotator());
		}
		else
		{
			participants_[1]->setPose(player_starts[0].GetLocation(), player_starts[0].GetRotation().Rotator());
			participants_[0]->setPose(player_starts[1].GetLocation(), player_starts[1].GetRotation().Rotator());
		}
	}
}

GateID URace::GateSuccessor(GateID predecessor) const
{
	auto successorElement = gate_successors_.find(predecessor);
	return successorElement != gate_successors_.end() ? successorElement->second : NO_GATE;
}

bool URace::IsLastGate(GateID gate) const
{
	return gate == final_gate_;
}

Time URace::TimeStamp()
{
	using namespace std::chrono;
	milliseconds ms = duration_cast<milliseconds>(
		system_clock::now().time_since_epoch()
	);
	return ms.count();
}

const std::vector<RacerProgress> URace::getProgress() const
{
	std::vector<RacerProgress> progresses;
	for (auto participant : participants_)
		progresses.push_back(participant->getProgress());
	return progresses;
}

const std::vector<Odometry> URace::getOdometry() const
{
	std::vector<Odometry> odometry;
	for (auto participant : participants_)
		odometry.push_back(participant->getOdometry());
	return odometry;
}

const std::vector<std::string> URace::getRacerNames() const
{
	std::vector<std::string> names;
	for (auto participant : participants_)
		names.push_back(TCHAR_TO_UTF8(*participant->name_));
	return names;
}