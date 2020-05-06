#pragma once
#include "RacingDefines.h"
#include "RacerProgress.h"
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Racer.generated.h"

class IRaceable;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_FourParams(FCollisionEvent, ARacer*, me, ECollision, collision_type, float, severity_multiplier, const AActor*, collidee);


UCLASS(BlueprintType)
class ARacer : public AActor
{

	GENERATED_BODY()

public:
	void setPose(const FVector& position, const FRotator& rotation);
	RacerProgress& getProgress();
	const Odometry getOdometry() const;
	void setActor(AActor *racer_actor);
	const AActor* getRacingActor();
	void Disqualify();
	void StartRacing();
	void StopRacing();
	void clearProgress();

private:
	void SetGhostMode(bool isGhost);
	AActor *racer_actor_;
	RacerProgress progress_;
    
public:
	FString name_;

public:
	UFUNCTION(BlueprintCallable)
		void broadcastCollisionEvent(ECollision collision_type, float severity_multiplier, const AActor* actor_collided_with);

	UPROPERTY(BlueprintAssignable, BlueprintCallable, Category = "Racing")
	FCollisionEvent collision_event_;
};