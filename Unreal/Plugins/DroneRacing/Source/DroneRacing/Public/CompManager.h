#pragma once
#include <map>
#include <fstream>
#include "GameFramework/Actor.h"
#include "RacingDefines.h"
#include "StateReporter.h"
#include "CompManager.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE(FLockEmUp);
class URace;
class URacer;
class URaceHUD;
class RaceLog;

UENUM(BlueprintType)
enum FINISH_REASON
{
	COMPLETE,
	RESET
};

UCLASS()
//UCLASS(BlueprintType)
class DRONERACING_API ACompManager : public AActor
{
	GENERATED_BODY()

public:
	enum COMP_STATUS
	{
		FIRST_IDLE,
		FIRST_RACE,
		SECOND_IDLE,
		SECOND_RACE,
		END
	};


public:
	ACompManager();
	~ACompManager();
	UFUNCTION(BlueprintCallable, Category = "Racing")
		void BeginComp();
	UFUNCTION(BlueprintCallable, Category = "Racing")
		void EndComp();
	UFUNCTION(BlueprintCallable, Category = "Racing")
		void BeginRace(int tier);
	UFUNCTION(BlueprintCallable, Category = "Racing")
		void EndRace(FINISH_REASON finish_reason = FINISH_REASON::COMPLETE );
	UFUNCTION(BlueprintCallable, Category = "Racing")
		void ResetRace();
	UFUNCTION(BlueprintPure, Category = "Racing")
		static ACompManager* getManager();

	UPROPERTY(BlueprintAssignable)
		FLockEmUp OnLockEmUp;

	UFUNCTION(BlueprintImplementableEvent, Category = "Racing")
		bool IsLoggingDisabled();

	UFUNCTION(BlueprintImplementableEvent, Category = "Racing")
		void UpdateRacerStateInAirSim(const FString &racer_name, bool disqualified, int last_gate_passed);

	virtual void Tick(float dt) override;

private:
	void log();
	void createHUD();
	const std::string whereAmI() const;
	const std::string getCurrentTime() const;
	void generateLog(int tier) const;

private:
	//std::map<Racer*, RacerProgress> race_progress_;

	UPROPERTY()
	URace *race_instance_;
	
	COMP_STATUS COMP_STATUS_ = COMP_STATUS::FIRST_IDLE;

	StateReporter race_report_;
	//static std::ofstream *LOGGER;
	RaceLog *LOGGER;
	bool is_racing_{ false };
	bool is_logging_{ false };
	TArray<FTransform> start_transforms_;

	UClass *widget_class_;
	UPROPERTY() URaceHUD *race_hud_;

	static ACompManager* KAREN;
	Time init_race_time_;
	Time race_time_{ 0 };

	Time max_time_ =  5 * 60 * 1000;  // 5 minute max time
};
