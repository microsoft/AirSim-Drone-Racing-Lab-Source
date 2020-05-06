#pragma once
#include "CoreMinimal.h"
#include <string>
#include "Race.h"
#include "Engine/World.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Kismet/GameplayStatics.h"
#include "RacingLibrary.generated.h"

UCLASS()
class UTrack : public UObject
{
	GENERATED_BODY()
};

UCLASS()
class URacingLib : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:

	UFUNCTION(Category = "Racing")
	static URace* CreateRace(UWorld* context, FVector start_block);

	//UFUNCTION(BlueprintCallable, Category = "Racing")
	//static void RegisterRacer(Racer *racer, FString name = TEXT(""))
	//{}

	UFUNCTION(BlueprintCallable, Category = "Racing")
	static void RegisterTrack(UTrack *track, FString name = TEXT(""))
	{}

	/*
	* Returns false if the race failed to start.
	*/
	UFUNCTION(BlueprintCallable, Category = "Racing")
	static bool StartRace()
	{
		return true; 
	}

	//UFUNCTION(BlueprintCallable, Category = "Racing")
	//static void DisqualifyRacer(Racer* racer)
	//{}

public:
	static void getGates(UWorld* context, TArray<AGate *> &gates);
	static TArray<AActor*> getRacers(UObject* context);
	static TArray<FTransform> getStartPositions(UObject* context);
	static TArray<AActor*> getAllActorsWithTag(UObject* context, FName tag);
	template<typename T>
	static TArray<AActor*> GetAllActorsOfClass(const UObject* context)
	{
		TArray<AActor*> foundActors;
		UGameplayStatics::GetAllActorsOfClass(context, T::StaticClass(), foundActors);
		return foundActors;
	}
private:
	std::string RaceStartErrMsg = "";
};