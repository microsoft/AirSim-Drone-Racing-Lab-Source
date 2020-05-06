#include "CompManager.h"
#include "RacingLibrary.h"
#include "Racer.h"
#include "RaceHUD.h"
#include "StateReporter.h"
#include "GameFramework/PlayerStart.h"
#include "ConstructorHelpers.h"
#include "RaceLog.h"
#include <fstream>
#include <ctime>


//std::ofstream *ACompManager::LOGGER = new std::ofstream();
ACompManager *ACompManager::KAREN = nullptr;

ACompManager* ACompManager::getManager()
{
	return KAREN;
}

ACompManager::ACompManager()
{
	KAREN = this;
	LOGGER = new RaceLog();
	static ConstructorHelpers::FClassFinder<UUserWidget> hud_widget_class(TEXT("WidgetBlueprint'/DroneRacing/UI/RaceUI'"));
	widget_class_ = hud_widget_class.Succeeded() ? hud_widget_class.Class : nullptr;
	SetActorTickEnabled(true);
}

ACompManager::~ACompManager()
{
	LOGGER->close();
}

void ACompManager::Tick(float dt)
{
	if (is_racing_)
	{
		std::vector<RacerProgress> progress = race_instance_->getProgress();
		std::vector<std::string> names = race_instance_->getRacerNames();
		std::vector<Odometry> odometry = race_instance_->getOdometry();
		Time current_time = race_instance_->TimeStamp() - init_race_time_;

		for (int i = 0; i < progress.size(); i++)
		{
			auto racer_prog = progress[i];
			auto name = names[i];
			UpdateRacerStateInAirSim(FString(name.c_str()), racer_prog.isDisqualified(), racer_prog.GetLastGate());
		}

		int finished_racer_count{ 0 };
		for (int i = 0; i < progress.size(); i++)
		{
			race_hud_->update(race_report_.update(race_instance_->getRacerNames()[i], progress[i], odometry[i], current_time));
			if (progress[i].isFinished())
			{
				finished_racer_count++;
				if (progress[i].isDisqualified())
					race_hud_->flashDisqualified(race_instance_->getRacerNames()[i]);
				else
					race_hud_->flashFinished(race_instance_->getRacerNames()[i]);
			}
		}
		if (is_logging_)
			log();

		if (finished_racer_count >= progress.size() || current_time >= max_time_)
			EndRace();
	}
}


void ACompManager::createHUD()
{
	if (widget_class_)
		race_hud_ = CreateWidget<URaceHUD>(this->GetWorld()->GetFirstPlayerController(), widget_class_);

}

void ACompManager::BeginComp()
{
	start_transforms_ = URacingLib::getStartPositions(this->GetWorld());
	race_instance_ = URacingLib::CreateRace(this->GetWorld(), (start_transforms_[0].GetLocation() + start_transforms_[1].GetLocation())/2.f);
	createHUD();
	if (start_transforms_.Num() >= 2)
	{
		race_instance_->placeRacersAtStartPositions(start_transforms_, true);
	}

	if (widget_class_)
	{
		race_hud_->TotalGates = race_instance_->NumberOfGates();
	}
}

void ACompManager::EndComp()
{
	is_logging_ = false;
	LOGGER->close();
	// Remove hud from viewport
	race_hud_->RemoveFromViewport();
	//destroy HUD
	race_hud_->Destruct();
}

const std::string ACompManager::whereAmI() const
{
	std::string level_name{""};
	for (auto tag : Tags)
		if (tag.ToString().Contains("level"))
			level_name = TCHAR_TO_UTF8(*tag.ToString());
	return level_name;
}
const std::string ACompManager::getCurrentTime() const
{
	auto t = std::time(nullptr);
	auto tm = *std::localtime(&t);
	std::stringstream ss;
	ss << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");

	return ss.str();
}
void ACompManager::generateLog(int tier) const
{
	std::string file_header =  getCurrentTime() + "_" + whereAmI()  + 
		"_tier_" + std::to_string(tier) + "_" + 
		std::to_string(race_hud_->RaceNumber) +".log";

	LOGGER->open(file_header);
	LOGGER->log( "HEADER Level " + whereAmI() + "\nHEADER Tier " + std::to_string(tier) + "\nHEADER RaceNumber " + 
		std::to_string(race_hud_->RaceNumber) + "\n");
}

void ACompManager::BeginRace(int tier)
{
	if (!race_hud_->IsInViewport())
		race_hud_->AddToViewport();
	if ( COMP_STATUS_ == COMP_STATUS::FIRST_IDLE || COMP_STATUS_ == COMP_STATUS::SECOND_IDLE )
	{
		//Reset progress representations in AirSim.
		std::vector<RacerProgress> progress = race_instance_->getProgress();
		std::vector<std::string> names = race_instance_->getRacerNames();
		for (int i = 0; i < progress.size(); i++)
		{
			auto racer_prog = progress[i];
			auto name = names[i];
			UpdateRacerStateInAirSim(FString(name.c_str()), false, -1);
		}

		race_instance_->start(tier);
		init_race_time_ = race_instance_->TimeStamp();
		COMP_STATUS_ = static_cast<COMP_STATUS>(int(COMP_STATUS_) + 1) ;
		race_time_ = 0;
		is_logging_ = !IsLoggingDisabled();
		is_racing_ = true;
		race_hud_->FlashMessage("Go!", FColor::Green);
	}

	if (is_logging_)
		generateLog(tier);
}

void ACompManager::EndRace(FINISH_REASON finish_reason)
{
	if (finish_reason == FINISH_REASON::RESET)
		race_hud_->FlashMessage("RACE RESET", FColor::Yellow);
	else if (COMP_STATUS_ == COMP_STATUS::FIRST_RACE || COMP_STATUS_ == COMP_STATUS::SECOND_RACE)
	{
		COMP_STATUS_ = static_cast<COMP_STATUS>(int(COMP_STATUS_) + 1);
		race_hud_->RaceNumber += 1;
	}


	race_hud_->reset();
	race_instance_->end();
	LOGGER->close();

	is_logging_ = false;
	is_racing_ = false;
	OnLockEmUp.Broadcast();
	//start_transforms_ = URacingLib::getStartPositions(this->GetWorld());
	race_instance_->placeRacersAtStartPositions(start_transforms_, COMP_STATUS_ != COMP_STATUS::SECOND_IDLE);
	if (COMP_STATUS_ == COMP_STATUS::END)
	{
		COMP_STATUS_ = COMP_STATUS::FIRST_IDLE;
		race_hud_->RaceNumber = 1;
	}
}

void ACompManager::ResetRace()
{
	EndRace(FINISH_REASON::RESET);
	race_hud_->RaceNumber = 1;
	COMP_STATUS_ = COMP_STATUS::FIRST_IDLE;

}

void ACompManager::log()
{
	LOGGER->log(race_report_.getOutput());
	race_report_.clear();
}