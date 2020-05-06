#include "Racer.h"
#include "RacingDefines.h"

void ARacer::broadcastCollisionEvent(ECollision collision_type, float severity_multiplier, const AActor* actor_collided_with)
{
	collision_event_.Broadcast(this, collision_type, severity_multiplier, actor_collided_with);
}

RacerProgress& ARacer::getProgress()
{
	return progress_;
}

void ARacer::clearProgress()
{
	progress_.clear();
}

const Odometry ARacer::getOdometry() const
{
	return Odometry
	{
		racer_actor_->GetActorLocation(),
		racer_actor_->GetActorRotation()
	};
}

void ARacer::setPose(const FVector& position, const FRotator& rotation)
{
	racer_actor_->SetActorLocationAndRotation(position, rotation, false, nullptr, ETeleportType::TeleportPhysics);
}

void ARacer::setActor(AActor *racer_actor)
{
	racer_actor_ = racer_actor;
	name_ = racer_actor->GetName();
}

const AActor* ARacer::getRacingActor()
{
	return racer_actor_;
}
void ARacer::Disqualify()
{
	progress_.Disqualify();
	SetGhostMode(true);
}

void ARacer::StartRacing()
{
	SetGhostMode(false);
}

void ARacer::StopRacing()
{
	SetGhostMode(false);
	clearProgress();
}

void ARacer::SetGhostMode(bool isGhost)
{
	racer_actor_->SetActorHiddenInGame(isGhost);
	racer_actor_->SetActorEnableCollision(!isGhost);
}