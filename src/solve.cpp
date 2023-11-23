#include "simulate.h"
#include <cmath>

double RoundaboutManager::calculate_reality_total_time() {
        double total_time = 0;
        
        for (Vehicle vec : _vehicles) {
        	double current_vehicle_travel_time = 0;
        	double travel_length;
        	int entry_index = vec.get_entry_index();
        	int exit_index = vec.get_exit_index();
        	if (exit_index > entry_index) {
        		travel_length = (exit_index - entry_index) * _sectionLength;
        	}
        	else {
        		travel_length = (exit_index + _numSection - entry_index) * _sectionLength;
        	}

		int cnt = 0;
		double previous_change_acceleration_time = 0;
		double previous_acceleration = 0;
		double already_travel_length = 0;
		double current_velocity = 0;
		double last_change_acceleration_time;
		double remain_travel_length;
		double remain_travel_time;
		std::map<double, double> change_acceleration_info = vec.get_change_acceleration_info();
        	for (auto info : change_acceleration_info) {
        		cnt += 1;
        		double change_acceleration_time = info.first;
        		double current_acceleration = info.second;
        		double time_diff = change_acceleration_time - previous_change_acceleration_time;

			already_travel_length += current_velocity * time_diff + 0.5 * previous_acceleration * time_diff * time_diff;
        		current_velocity += previous_acceleration * time_diff;

        		if (cnt == change_acceleration_info.size()) {
        			last_change_acceleration_time = change_acceleration_time;
        			remain_travel_length = travel_length - already_travel_length;
        			remain_travel_time = (sqrt(current_velocity * current_velocity + 2 * current_acceleration * remain_travel_length) - current_velocity) / current_acceleration;
        			// S = v_0 * t + 0.5 * a * t ^ 2
        		}
        		else {
        			previous_change_acceleration_time = change_acceleration_time;
        			previous_acceleration = current_acceleration;
        		}
        	}
        	
        	current_vehicle_travel_time = last_change_acceleration_time + remain_travel_time;
		total_time += current_vehicle_travel_time;
        }
        
        
	return total_time;
}

double RoundaboutManager::calculate_ideal_total_time() {
	double total_time = 0;
        for (Vehicle vec : _vehicles) {
        	double travel_length;
        	int entry_index = vec.get_entry_index();
        	int exit_index = vec.get_exit_index();
        	if (exit_index > entry_index) {
        		travel_length = (exit_index - entry_index) * _sectionLength;
        	}
        	else {
        		travel_length = (exit_index + _numSection - entry_index) * _sectionLength;
        	}
        	
        	total_time += sqrt(2 * travel_length / _maxAllowedVelocity);
        }
        
	return total_time;
} 
