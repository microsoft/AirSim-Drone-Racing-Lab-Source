#pragma once
#include "RacingDefines.h"
#include "CoreMinimal.h"
#include "Gate.generated.h"

class URace;

UCLASS(BlueprintType)
class AGate : public AActor
{
	GENERATED_BODY()

public:
	UPROPERTY(BlueprintReadOnly)
	int id_;

	void registerGatePassThroughHandler(URace *race);

protected:
	UFUNCTION(BlueprintCallable, Category = "Racing")
	void BroadcastPassThrough(int gate_id, const AActor *passer);

private:
	UPROPERTY()
	URace *gatePassHandler_;
};