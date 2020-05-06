#pragma once
#include "RaceHUD.h"

void URaceHUD::update(const State& current_state)
{

	//GateNumber = current_state.gates_passed;
	//Penalty = FString(current_state.penalty_string().c_str());
	Time = FString(current_state.time_string().c_str());
	updateStatusList(FString(current_state.name.c_str()), current_state.gates_passed, FString(current_state.penalty_string().c_str()));

}

void URaceHUD::flashFinished(const std::string& racer_name)
{
	if (!has_won_)
	{
		std::string message = racer_name + " finished!";
		FlashMessage(message.c_str(), FColor::Green);
		has_won_ = true;
	}
}

void URaceHUD::flashDisqualified(const std::string& racer_name)
{
	if (!has_dqed_)
	{
		std::string message = racer_name + " was DQed!";
		FlashMessage(message.c_str(), FColor::Red);
		has_dqed_ = true;
	}
}

void URaceHUD::reset()
{
	has_dqed_ = false;
	has_won_ = false;
	clearStatusList();
}