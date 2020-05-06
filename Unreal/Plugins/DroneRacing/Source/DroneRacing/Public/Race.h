#pragma once
#include "Object.h"
#include "RacingDefines.h"
#include "Racer.h"
#include <vector>
#include "CoreMinimal.h"
#include <chrono>
#include "Race.generated.h"

class UWorld;
class AGate;

UCLASS(BlueprintType)
class URace : public UObject
{
	GENERATED_BODY()

public:
	static Time TimeStamp();

	GateID GateSuccessor(GateID predecessor) const;
	void placeRacersAtStartPositions(const TArray<FTransform>& player_starts, bool is_first_race);
	void initialize(UWorld* world_context, const FVector start_block);
	bool IsLastGate(GateID gate) const;
	int NumberOfGates() const;
	TArray<ARacer*> convert(const TArray<AActor*>& race_actors);
	
	UFUNCTION()
	void onGatePassedThrough(int id, const AActor *passer);
	const std::vector<RacerProgress> getProgress() const;
	const std::vector<Odometry> getOdometry() const;
	const std::vector<std::string> getRacerNames() const;
	void start(int tier);
	void end();

private:
	void orderGates();
	void subscribeToGatesEvents();
	void subscribeToCollisionEvents();
	UFUNCTION()
	void Penalize(ARacer* racer, ECollision collision_type, float severity_multiplier, const AActor* actor_collided_with);
	bool droneIsFollower(ARacer* drone, ARacer* collided_drone);
	float normDistance(const AActor* drone, const GateID& gate_id);
private:
	GateID final_gate_;
	UPROPERTY()
	FVector start_block_;
	UPROPERTY()
	TArray<AGate*> gates_;
	std::map<GateID, GateID> gate_successors_;
	UPROPERTY()
	TArray<ARacer*> participants_;

	UPROPERTY()
	UWorld* world_;
	int number_of_gates_{ 0 };
	UPROPERTY()
	ARacer *leader_ = nullptr;

	bool is_racing_{ false };

};