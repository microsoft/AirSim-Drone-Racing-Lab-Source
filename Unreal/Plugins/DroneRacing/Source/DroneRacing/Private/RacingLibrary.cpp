#include "RacingLibrary.h"
#include "Gate.h"
#include "EngineUtils.h"
#include <regex>

URace* URacingLib::CreateRace(UWorld* world_context, FVector start_block)
{
	URace* new_race = NewObject<URace>();
	new_race->initialize(world_context, start_block);
	return new_race;
}

void URacingLib::getGates(UWorld* world, TArray<AGate *> &gates)
{
	gates.Empty();
	for (TActorIterator<AGate> actor_itr(world); actor_itr; ++actor_itr)
	{
		AGate* gate = *actor_itr;

		//Determine number in gate ordering.
		std::string gate_name = TCHAR_TO_UTF8(*gate->GetName());
		std::string number_extension = std::regex_replace(
			gate_name,
			std::regex("[^0-9]*([0-9]+).*"),
			"$1"
		);
		if (number_extension != "")
		{
			gate->id_ = std::stoi(number_extension);
		}

		//Add to list.
		gates.Add(gate);
	}

	gates.Sort([](const AGate& a, const AGate& b) { return a.id_ < b.id_; });
}

TArray<AActor*> URacingLib::getRacers(UObject* context)
{
	auto racers = getAllActorsWithTag(context, "airsim_drone");
	return racers;
}
TArray<FTransform> URacingLib::getStartPositions(UObject* context)
{
	TArray<FTransform> start_transforms;
	auto start_block = getAllActorsWithTag(context, "start_block")[0];
	for (auto start_object : start_block->GetComponentsByTag(UActorComponent::StaticClass(), "race_start"))
		start_transforms.Add(dynamic_cast<USceneComponent*>(start_object)->GetComponentTransform());
	return start_transforms;

}

TArray<AActor*> URacingLib::getAllActorsWithTag(UObject* context, FName tag)
{
	TArray<AActor*> matching_actors;
	UGameplayStatics::GetAllActorsWithTag(context, tag, matching_actors);
	return matching_actors;
}