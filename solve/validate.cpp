#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdexcept>
#include "constants.hpp"


class Vehicle 
{
public:
    Vehicle(int id, int entry_section, int exit_section, double arrival_time, double init_velocity)
    {
        _id = id;
        _entry_section = entry_section;
        _exit_section = exit_section;
        _arrival_time = arrival_time;
        _current_velocity = init_velocity;
    }

    int _id;
    int _entry_section;
    int _exit_section;
    double _arrival_time;
    double _exit_time;
    double _current_velocity;
    std::vector<double> _times;
	std::vector<double> _accelerations;
	std::vector<double> _velocities;
};


int num_vehicle;
int num_section;
std::vector<double> section_lengths;
std::vector<Vehicle> vehicles;


void check_input_file_is_valid_format(std::string filename) {
    std::ifstream file(filename);
    if (!file.is_open()) EXIT("File " + filename + " cannot be opened.");
    std::string line;
    if (!std::getline(file, line)) EXIT("Cannot read the first line of " + filename);
    std::stringstream ss(line);
    if (!(ss >> num_vehicle >> num_section)) EXIT("Invalid format in the first line of " + filename);
    if (num_vehicle <= 0) EXIT("Number of vehicles is not positive.");
    if (num_section <= 0) EXIT("Number of sections is not positive.");
    try {
        for (int i = 0; i < num_section; i++) {
            double section_length;
            if (!(ss >> section_length)) EXIT("Not enough section lengths in the first line.");
            if (section_length < 0) EXIT("Section length cannot be negative");
            section_lengths.push_back(section_length);
        }
    } catch (const std::invalid_argument&) {
        EXIT("Invalid format for section length");
    } catch (const std::out_of_range&) {
        EXIT("Section length value out of range");
    }

    for (int i = 0 ; i < num_vehicle ; i++) {
        if (!std::getline(file, line)) EXIT("Not enough lines for vehicle data");
        std::stringstream ss(line);
        int id, entry_section, exit_section;
        double arrival_time, init_velocity;
        if (!(ss >> id >> entry_section >> exit_section >> arrival_time >> init_velocity)) EXIT("Invalid format for vehicle data");
        Vehicle vehicle(id, entry_section, exit_section, arrival_time, init_velocity);
        vehicles.push_back(vehicle);
        if (!ss.good() || !(ss >> std::ws).eof()) EXIT("Extra characters after valid data in vehicle data line");
    }
}


void check_output_file_is_valid_format(std::string filename) {
    std::ifstream file(filename);
    if (!file.is_open()) EXIT("File " + filename + " cannot be opened.");
    for (int i = 0 ; i < num_vehicle ; i++) {
        std::string line;
        if (!std::getline(file, line)) EXIT("Not enough lines for vehicle data");
        std::stringstream ss(line);
        
        int id;
        double exit_time;
        int num_data_points = 0;
        if (!(ss >> id >> exit_time)) EXIT("Invalid format for output vehicle data");
        for (Vehicle v : vehicles) {
            if (v._id == id) {
                v._exit_time = exit_time;
                break;
            }
        }
        
        while (ss >> std::ws) {
            double time, acceleration;
            if (!(ss >> time >> acceleration)) EXIT("Invalid format for output vehicle data");
            for (Vehicle v : vehicles) {
                if (v._id == id) {
                    v._times.push_back(time);
                    v._accelerations.push_back(acceleration);
                    break;
                }
            }
            num_data_points++;
        }

        if (num_data_points % 2 != 0) EXIT("Time and acceleration doesn't pair up.");
    }
}


void check_exit_time_bigger_than_arrival_time() {
    for (Vehicle v : vehicles) {
        if (v._exit_time < v._arrival_time) EXIT("Vehicle id " + std::to_string(v._id) + " arrival time " + std::to_string(v._arrival_time) + " is bigger than exit time " + std::to_string(v._exit_time));
    }
}


void check_acceleration_constraint() {
    for (Vehicle v : vehicles) {
        for (const auto & a : v._accelerations) {
            if (a > MAX_A) EXIT("Vehicle id " + std::to_string(v._id) + " acceleration " + std::to_string(a) + " is over maximum " + std::to_string(MAX_A));
            if (a < MIN_A) EXIT("Vehicle id " + std::to_string(v._id) + " acceleration " + std::to_string(a) + " is under minimum " + std::to_string(MIN_A));
        }
    }
}


void check_velocity_constraint() {
    for (Vehicle v : vehicles) {
        if (v._current_velocity > MAX_V) EXIT("Vehicle id " + std::to_string(v._id) + " initial velocity " + std::to_string(v._current_velocity) + " is over maximum " + std::to_string(MAX_V));
        v._velocities.push_back(v._current_velocity);
        for (int i = 1 ; i < v._times.size() ; i++) {
            double time_period = v._times[i] - v._times[i - 1];
            v._current_velocity += time_period * v._accelerations[i - 1];
            if (v._current_velocity > MAX_V) EXIT("Vehicle id " + std::to_string(v._id) + " velocity " + std::to_string(v._current_velocity) + " is over maximum " + std::to_string(MAX_V));
            v._velocities.push_back(v._current_velocity);
        }

        double time_period = v._exit_time - v._times[v._times.size() - 1];
        v._current_velocity += time_period * v._accelerations[v._times.size() - 1];
        if (v._current_velocity > MAX_V) EXIT("Vehicle id " + std::to_string(v._id) + " velocity " + std::to_string(v._current_velocity) + " is over maximum " + std::to_string(MAX_V));
        v._velocities.push_back(v._current_velocity);   
    }
}


void solve_quardric_equation(double v1, double v2, double a1, double a2, double time_start, double time_end) {
    double check_1 = (v1 - v2 - a2 * TIME_GAP) * (v1 - v2 - a2 * TIME_GAP) + (a1 - a2) * (a2 * TIME_GAP * TIME_GAP + 2 * v2 * TIME_GAP);
    if (check_1 >= 0) {
        double t1 = ((v2 + a2 * TIME_GAP - v1) + sqrt(check_1)) / (a1 - a2);
        double t2 = ((v2 + a2 * TIME_GAP - v1) - sqrt(check_1)) / (a1 - a2);
        if (t1 >= time_start && t1 <= time_end) EXIT("collision");
        if (t2 >= time_start && t2 <= time_end) EXIT("collision");
    }

    double check_2 = (v1 - v2 + a2 * TIME_GAP) * (v1 - v2 + a2 * TIME_GAP) + (a1 - a2) * (a2 * TIME_GAP * TIME_GAP - 2 * v2 * TIME_GAP);
    if (check_2 >= 0) {
        double t1 = ((v2 - a2 * TIME_GAP - v1) + sqrt(check_2)) / (a1 - a2);
        double t2 = ((v2 - a2 * TIME_GAP - v1) - sqrt(check_2)) / (a1 - a2);
        if (t1 >= time_start && t1 <= time_end) EXIT("collision");
        if (t2 >= time_start && t2 <= time_end) EXIT("collision");
    }
}


void check_safety_time_margin() {
    for (Vehicle first_vehicle : vehicles) {
        for (int i = 1 ; i < first_vehicle._times.size() ; i++) {
            double start_time = first_vehicle._times[i - 1];
            double end_time = first_vehicle._times[i];
            double v = first_vehicle._velocities[i - 1];
            double a = first_vehicle._accelerations[i - 1];
            for (Vehicle second_vehicle : vehicles) {
                if (first_vehicle._id == second_vehicle._id) continue;
                if (start_time >= second_vehicle._exit_time) continue;
                if (end_time <= second_vehicle._arrival_time) continue;
                int overlap_start = -1;
                int overlap_end = -1;
                for (int j = 1 ; j < second_vehicle._times.size() ; j++) {
                    if (second_vehicle._times[j - 1] <= start_time && second_vehicle._times[j] >= start_time) {
                        overlap_start = j - 1;
                        break;
                    }
                }

                for (int j = 1 ; j < second_vehicle._times.size() ; j++) {
                    if (second_vehicle._times[j - 1] <= end_time && second_vehicle._times[j] >= end_time) {
                        overlap_end = j - 1;
                        break;
                    }
                }

                if (overlap_start != -1 && overlap_end != -1) {
                    for (int k = overlap_start ; k <= overlap_end ; k++) {
                        double time_upper_bound;
                        double time_lower_bound;
                        if (k + 1 >= second_vehicle._times.size()) {
                            time_upper_bound = second_vehicle._exit_time;
                            time_lower_bound = start_time;
                        }
                        else if (k == overlap_start) {
                            time_upper_bound = second_vehicle._times[k + 1];
                            time_lower_bound = start_time;
                        } else if (k == overlap_end) {
                            time_upper_bound = end_time;
                            time_lower_bound = second_vehicle._times[k];
                        } else {
                            time_upper_bound = second_vehicle._times[k + 1];
                            time_lower_bound = second_vehicle._times[k];
                        }

                        solve_quardric_equation(v, second_vehicle._velocities[k], a, second_vehicle._accelerations[k], time_lower_bound, time_upper_bound);
                    }
                }
            }
        }
    }
}


void check_finish_the_track() {
    for (Vehicle v : vehicles) {
        double expected_travel_length = 0;
		double actual_travel_length = 0;

        // Calculate Expected Travel Length
        for (int i = v._entry_section ; i < v._exit_section ; i++) {
			expected_travel_length += section_lengths[i];
		}

        // Check Actual Travel Length
        for (int i = 1 ; i < v._times.size() ; i++) {
			double time_period = v._times[i] - v._times[i - 1];
			actual_travel_length += v._velocities[i - 1] * time_period + 0.5 * v._accelerations[i - 1] * time_period * time_period;
		}

        double time_period = v._exit_time - v._times[v._times.size() - 1];
        actual_travel_length += v._velocities[v._times.size() - 1] * time_period + 0.5 * v._accelerations[v._times.size() - 1] * time_period * time_period;
        if (actual_travel_length != expected_travel_length) EXIT("Vehicle id " + std::to_string(v._id) + " expect to travel " + std::to_string(expected_travel_length) + " but actual travel " + std::to_string(actual_travel_length));
    }
}


int main(int argc, char* argv[]) {
	if (argc != 3) EXIT("Should have three command line arguments input.");

	std::string input_filename = argv[1];
	std::string output_filename = argv[2];

	check_input_file_is_valid_format(input_filename);
	check_output_file_is_valid_format(output_filename);
	check_exit_time_bigger_than_arrival_time();
	check_acceleration_constraint();
	check_velocity_constraint();
	check_safety_time_margin();
	check_finish_the_track();
}