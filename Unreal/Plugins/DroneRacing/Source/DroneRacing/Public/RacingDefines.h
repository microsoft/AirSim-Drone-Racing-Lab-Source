#pragma once
#include "CoreMinimal.h"
#include <map>

class ARacer;

typedef uint16_t GateID;
typedef uint64_t Time;

const GateID NO_GATE = 0xFFFF;
const FString BASELINE_NAME = "drone_2";
UENUM(BlueprintType)
enum ECollision
{
	SafetyViolation,
	DroneCollision,
	EnvironmentCollision
};

struct FCollisionPenalty
{
	ECollision collision_type;
	Time time_occurred;
	float severity_multiplier;
	AActor *collider;

	friend bool operator< (const FCollisionPenalty& l, const FCollisionPenalty& r) { return (l.time_occurred < r.time_occurred) || ((l.time_occurred == r.time_occurred) && (l.collision_type < r.collision_type)); }

	FCollisionPenalty(const ECollision& collision_type, const Time& time, const float& severity, AActor *collide = nullptr) :
		collision_type(collision_type), time_occurred(time), severity_multiplier(severity) 
		{collider = collide;};
};

struct Odometry
{
	FVector location;
	FRotator rotation;
};