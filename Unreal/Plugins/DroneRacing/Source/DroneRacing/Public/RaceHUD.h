#pragma once
#include "CoreMinimal.h"
#include "StateReporter.h"
#include "Blueprint/UserWidget.h"
#include "RaceHUD.generated.h"


UCLASS()
class DRONERACING_API URaceHUD : public UUserWidget
{
	GENERATED_BODY()

public:
	//UFUNCTION(BlueprintImplementableEvent, Category = "Racing | UI")
	void update(const State& current_state);

	UPROPERTY(BlueprintReadOnly, Category = "Racing | UI")
		int Place{ 1 };
	UPROPERTY(BlueprintReadOnly, Category = "Racing | UI")
		int TotalCompetitors{ 2 };
	//UPROPERTY(BlueprintReadOnly, Category = "Racing | UI")
	//	int GateNumber{ 0 };
	UPROPERTY(BlueprintReadOnly, Category = "Racing | UI")
		int TotalGates;
	UPROPERTY(BlueprintReadOnly, Category = "Racing | UI")
		FString Time{"00:00"};
	//UPROPERTY(BlueprintReadOnly, Category = "Racing | UI")
	//	FString Penalty{"00:00"};
	UPROPERTY(BlueprintReadOnly, Category = "Racing | UI")
		int RaceNumber{ 1 };
	UFUNCTION(BlueprintImplementableEvent, Category = "Racing | UI")
		void FlashMessage(const FString& message, const FColor& color);

	UFUNCTION(BlueprintImplementableEvent, Category = "Racing | UI")
		void updateStatusList(const FString& racer_name, const int& gates_passed, const FString& penalty_time);
	UFUNCTION(BlueprintImplementableEvent, Category = "Racing | UI")
		void clearStatusList();


	void flashFinished(const std::string& racer_name);
	void flashDisqualified(const std::string& racer_name);
	void reset();

private:
	bool has_won_{ false };
	bool has_dqed_{ false };
};