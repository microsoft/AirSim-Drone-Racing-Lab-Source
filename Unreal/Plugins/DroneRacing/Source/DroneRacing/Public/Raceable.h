#pragma once
#include "Object.h"
#include "Raceable.generated.h"

class ARacer;

UINTERFACE()
class URaceable : public UInterface
{
	GENERATED_UINTERFACE_BODY()

};

class IRaceable
{
	GENERATED_IINTERFACE_BODY()

public:
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = Racing)
	void AssignRacer(ARacer *racer);

	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = Racing)
	ARacer *GetRacer();
};