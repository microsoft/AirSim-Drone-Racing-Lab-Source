#pragma once
#include <sstream>
#include <string>
#include <iomanip>
#include "Racer.h"
#include "RacerProgress.h"

struct State
{
	std::string name;
	Odometry odometry;
	int gates_passed;
	int gates_missed;
	bool safety_violation{ false };
	int collision_penalties{ 0 };
	Time time_milliseconds;
	Time penalty_milliseconds;
public:

	static const State EMPTY_STATE;

	State() {}

	State(const std::string& n, const Odometry& o, int passed, int missed, bool violation,
		int penalties, Time t_ms, Time p_ms) : name(n), odometry(o), gates_passed(passed), gates_missed(missed), safety_violation(violation),
		collision_penalties(penalties), time_milliseconds(t_ms), penalty_milliseconds(p_ms)
	{}

	State(const std::string& name, const RacerProgress& progress, const Odometry& odom, const Time& time) :
		name(name), odometry(odom), gates_passed(progress.gatesPassed().size()),
		gates_missed(progress.gatesSkipped().size()),
		collision_penalties(progress.collisionPenalties().size()),
		time_milliseconds(time), penalty_milliseconds(progress.penaltyTime()) {}

	const std::string time_string() const
	{
		//return std::to_string(time_milliseconds / 60) + ":" +  std::to_string( time_milliseconds - int(float(time_milliseconds) / 60.0)*60 );
		return clock_string(time_milliseconds);
	}
	const std::string penalty_string() const
	{
		//return std::to_string(penalty_milliseconds % 60) + ":" + std::to_string(penalty_milliseconds - (penalty_milliseconds % 60));
		return "+" + clock_string(penalty_milliseconds);
	}

	State operator-(State a)
	{
		State diff
		{
			name,
			odometry,
			gates_passed - a.gates_passed,
			gates_missed - a.gates_missed,
			safety_violation && !a.safety_violation,
			collision_penalties - a.collision_penalties,
			time_milliseconds - a.time_milliseconds,
			penalty_milliseconds - a.penalty_milliseconds
		};
		return diff;
	}

private:
	const std::string clock_string(const Time& time) const
	{
		//int minutes = time / 60;
		//int seconds = time - int(float(time) / 60.0) * 60;

		//std::string minutes_string = minutes > 9 ? std::to_string(minutes) : "0" + std::to_string(minutes);
		//std::string seconds_string = seconds > 9 ? std::to_string(seconds) : "0" + std::to_string(seconds);

		//return minutes_string + ":" + seconds_string;

		std::string second_string = std::to_string(time);
		if (second_string.size() >= 3 )
			second_string.insert(second_string.end() - 3, '.');
		return second_string;
	}
};


class StateReporter
{
public:
	StateReporter(int float_precision = 3, bool is_scientific_notation = false)
	{
		initialize(float_precision, is_scientific_notation);
	}
	void initialize(int float_precision = 3, bool is_scientific_notation = false)
	{
		float_precision_ = float_precision;
		is_scientific_notation_ = is_scientific_notation;

		if (float_precision_ >= 0) {
			ss_ << std::setprecision(float_precision_);
			if (is_scientific_notation_)
				ss_ << std::scientific;
			else
				ss_ << std::fixed;
		}
	}
	void clear()
	{
		ss_.str(std::string());
		ss_.clear();
	}

	std::string getOutput() const
	{
		return ss_.str();
	}


	State update(std::string racer_name, const RacerProgress& racer_progress, const Odometry& odometry, const Time& time)
	{
		State latest_state(racer_name, racer_progress, odometry, time);
		State diff = latest_state - previous_state_;
		writeValue(racer_name, time, latest_state.odometry);
		if (diff.gates_passed > 0)
			writeValue(racer_name, time, "gates_passed", latest_state.gates_passed);
		if (diff.gates_missed > 0)
			writeValue(racer_name, time, "gates_missed", latest_state.gates_missed);
		if (diff.collision_penalties > 0)
			writeValue(racer_name, time, "collision_count", latest_state.collision_penalties);
		if (diff.penalty_milliseconds > 0)
			writeValue(racer_name, time, "penalty", latest_state.penalty_milliseconds);
		if (racer_progress.isDisqualified())
			writeValue(racer_name, time, "disqualified", racer_progress.isDisqualified());
		if (racer_progress.isFinished())
			writeValue(racer_name, time, "finished", racer_progress.isFinished());

		previous_state_ = latest_state;
		return latest_state;
	}

private:
	State previous_state_ = State::EMPTY_STATE;

	void startHeading(std::string heading, unsigned int heading_size, unsigned int columns = 20)
	{
		ss_ << "\n";
		ss_ << heading;
	}
	void endHeading(bool end_line, unsigned int heading_size, unsigned int columns = 20)
	{
		if (end_line)
			ss_ << "\n";
		for (int lines = heading_size; lines > 0; --lines)
			ss_ << std::string(columns, '_') << "\n";
	}
	void writeHeading(std::string heading, unsigned int heading_size = 0, unsigned int columns = 20)
	{
		startHeading(heading, heading_size);
		endHeading(true, heading_size, columns);
	}

	void writeValue(const std::string& racer_name, const Time time, const Odometry& odom)
	{
		ss_ << racer_name << " time " << time << " odometry_XYZRPY (" << odom.location.X << "," << odom.location.Y << "," << odom.location.Z << ","
			<< odom.rotation.Roll << "," << odom.rotation.Pitch << "," << odom.rotation.Yaw << ")\n";
	}
	template<typename T>
	void writeValue(const std::string& racer_name, const Time time, const std::string& value_name, const T& single_value)
	{
		ss_ << racer_name << " time " << time << " " << value_name << " " << single_value << "\n";
	}
	void writeNameOnly(std::string value_name)
	{
		ss_ << value_name << " ";
	}
	template<typename T>
	void writeValueOnly(const T& r, bool end_line_or_tab = false)
	{
		ss_ << r;

		if (end_line_or_tab)
			ss_ << "\n";
		else
			ss_ << "\t";
	}
	void endl()
	{
		ss_ << std::endl;
	}

private:
	std::stringstream ss_;

	int float_precision_ = 2;
	bool is_scientific_notation_ = false;
};

