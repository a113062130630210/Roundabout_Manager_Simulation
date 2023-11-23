#ifndef _SIMULATE_H
#define _SIMULATE_H

#include <vector>
#include <map>

class Vehicle
{
public:
	// Constructor
	Vehicle(int id, int entryIndex, int exitIndex, double earlistArrivalTime)
		: _id(id), _entryIndex(entryIndex), _exitIndex(exitIndex), _earlistArrivalTime(earlistArrivalTime)
	{
		_velocity = 0;
		_initVelocity = 0;
	};

	int get_id() {return _id;}
	int get_entry_index() {return _entryIndex;}
	int get_exit_index() {return _exitIndex;}

	double get_velocity() {return _velocity;}
	double get_init_velocity() {return _initVelocity;}
	double get_acceleration() {return _acceleration;}
	double get_earliest_arrival_time() {return _earlistArrivalTime;}
	double get_scheduled_enter_time() {return _scheduledEnterTime;}
	
	std::map<double, double> get_change_acceleration_info() {return _changeAccelerationInfo;}

private:
	int _id;
	int _entryIndex;
	int _exitIndex;

	double _velocity;
	double _initVelocity;
	double _acceleration;
	double _earlistArrivalTime;
	double _scheduledEnterTime;
	
	std::map<double, double> _changeAccelerationInfo;
	// key is time, value is acceleration
	// Assume that the acceleration is the same between two consecutive keys
};


class RoundaboutManager
{
public:
	// Constructor
	RoundaboutManager(int numVehicle, int numSection, double sectionLength, double maxAllowedVelocity, std::vector<Vehicle>& vehicles)
		: _numVehicle(numVehicle), _numSection(numSection), _sectionLength(sectionLength), _maxAllowedVelocity(maxAllowedVelocity), _vehicles(vehicles)
	{};

	int get_num_vehicle() {return _numVehicle;}
	int get_num_section() {return _numSection;}

	double calculate_reality_total_time();
	double calculate_ideal_total_time();
	double output() {return calculate_reality_total_time() - calculate_ideal_total_time();}

	double get_section_length() {return _sectionLength;}
	double get_max_allowed_velocity() {return _maxAllowedVelocity;}

private:
	int _numVehicle;
	int _numSection;
	double _sectionLength;
	double _maxAllowedVelocity;
	std::vector<Vehicle> _vehicles;
};

#endif
