#include <fstream>
#include <string>
#include <vector>

#include "constants.hpp"


// assume there are two files, input and output file

// input file has <num_vehicles> + 1 line
// the first line has <num_vehicles>, <num_sections>, and the length of each section
// then in the following <num_vehicles> each line has <vehicle_id>, <entry_section>, <exit_section>, <arrival_time>, <init_velocity>


// output file has <num_vehicles> line
// each line has 2k + 2 numbers, the first number is <vehicle_id>, the second number is <exit_time>, 
// after that the odd number is the time, and the even number is the acceleration


class Vehicle
{
public:
	// Constructor
	Vehicle(int id, int entryIndex, int exitIndex, double earlistArrivalTime, double initVelocity)
		: id(id), entryIndex(entryIndex), exitIndex(exitIndex), earlistArrivalTime(earlistArrivalTime), initVelocity(initVelocity)
	{
		velocity = initVelocity;
	};

	int id;
	int entryIndex;
	int exitIndex;

	double velocity;
	double initVelocity;
	double earlistArrivalTime;
	double exitTime;
	
	std::vector<double> times;
	std::vector<double> accelerations;
	std::vector<double> velocities;
	// Assume that the acceleration is the same between two consecutive time
};


int num_vehicle;
int num_section;
std::vector<double> section_lengths;
std::vector<Vehicle> vehicles;


void check_file_exists(std::string filename) {
	std::ifstream file;
	file.open(filename);
	if (!file) EXIT("filename " + filename + " doesn't exist.");
}


bool check_string_is_int(std::string s) {
	for (int i = 0 ; i < s.length() ; i++) {
		if (!isdigit(s[i])) return false;
	}
	return true;
}

bool check_string_is_double(std::string s) {
	for (int i = 0 ; i < s.length() ; i++) {
		if (!isdigit(s[i]) && s[i] != '.') return false;
	}
	return true;
}


// void check_first_line_of_input_file(std::string line) {
	
// }

void check_input_file_is_valid_format(std::string input_filename) {
	check_file_exists(input_filename);
	std::ifstream input_file (input_filename);
	std::string line;
	if (input_file.is_open()) {
		// process first line
		std::getline (input_file, line);
		std::vector<std::string> splitted_string;
		while (line.find(" ") != -1) {
			splitted_string.push_back(line.substr(0, line.find(" ")));
			line = line.substr(line.find(" ") + 1, line.length());
		}
		splitted_string.push_back(line);
		if (splitted_string.size() < 3) EXIT("Should have at least one section in input.");

		if (!check_string_is_int(splitted_string[0])) EXIT("Number of vehicles " + splitted_string[0] + " is not an integer.");
		num_vehicle = std::stoi(splitted_string[0]);
		if (num_vehicle <= 0) EXIT("Should have at least one vehicle, but get " + splitted_string[0]);

		if (!check_string_is_int(splitted_string[1])) EXIT("Number of sections " + splitted_string[1] + " is not an integer.");
		num_section = std::stoi(splitted_string[1]);
		if (num_section <= 0) EXIT("Should have at least one section, but get " + splitted_string[1]);

		if (num_section != splitted_string.size() - 2) EXIT("Expected to have " + splitted_string[1] + " sections, but get " + std::to_string(splitted_string.size() - 2));
		for (int i = 2 ; i < splitted_string.size() ; i++) {
			if(!check_string_is_double(splitted_string[i])) EXIT("Section length " + splitted_string[i] + " is not a double.");
			double section_length = std::stod(splitted_string[i]);
			if (section_length < 0) EXIT("Section length " + splitted_string[i] + " is less than zero.");
			section_lengths.push_back(section_length);
		}
		// finish process first line

		// process each line
		int counter = 0;
		while (std::getline(input_file, line)) {
			counter++;
			std::vector<std::string> tmp;
			while (line.find(" ") != -1) {
				tmp.push_back(line.substr(0, line.find(" ")));
				line = line.substr(line.find(" ") + 1, line.length());
			}
			tmp.push_back(line);
			if (tmp.size() != 5) EXIT("Each line should have five inputs, but line " + std::to_string(counter) + " only has " + std::to_string(tmp.size()));
			if (!check_string_is_int(tmp[0])) EXIT("At line " + std::to_string(counter) + " Vehicle ID should be an integer, but get " + tmp[0]);
			if (!check_string_is_int(tmp[1])) EXIT("At line " + std::to_string(counter) + " Entry index should be an integer, but get " + tmp[1]);
			if (!check_string_is_int(tmp[2])) EXIT("At line " + std::to_string(counter) + " Exit index should be an integer, but get " + tmp[2]);
			if (!check_string_is_double(tmp[3])) EXIT("At line " + std::to_string(counter) + " Earliest arrival time should be a double, but get " + tmp[3]);
			if (!check_string_is_double(tmp[4])) EXIT("At line " + std::to_string(counter) + " Init Velocity should be a double, but get " + tmp[4]);
			for (Vehicle v : vehicles) {
				if (v.id == std::stoi(tmp[0])) EXIT("At line " + std::to_string(counter) + " vehicle id " + tmp[0] + " already exists.");
			}
			Vehicle vehicle(std::stoi(tmp[0]), std::stoi(tmp[1]), std::stoi(tmp[2]), std::stod(tmp[3]), std::stod(tmp[4]));
			vehicles.push_back(vehicle);
		}
		if (counter != num_vehicle) EXIT("Expected to have " + std::to_string(num_vehicle) + " vehicles in input file, but get " + std::to_string(counter));
		// finish process each line
	}
}

void check_output_file_is_valid_format(std::string output_filename) {
	check_file_exists(output_filename);
	std::ifstream output_file (output_filename);
	std::string line;
	if (output_file.is_open()) {
		int counter = 0;
		while (std::getline(output_file, line)) {
			counter++;
			std::vector<std::string> tmp;
			while (line.find(" ") != -1) {
				tmp.push_back(line.substr(0, line.find(" ")));
				line = line.substr(line.find(" ") + 1, line.length());
			}
			tmp.push_back(line);
			if (tmp.size() < 1) EXIT("At line " + std::to_string(counter) + " expect to have at least two outputs.");
			if (!check_string_is_int(tmp[0])) EXIT("At line " + std::to_string(counter) + " vehicle_id " + tmp[0] + " is not an integer.");
			
			int vehicle_id = std::stoi(tmp[0]);
			int vehicle_index_in_vector = -1;
			for (int i = 0 ; i < vehicles.size() ; i++) {
				if (vehicles[i].id == vehicle_id) {
					vehicle_index_in_vector = i;
					break;
				}
			}
			if (vehicle_index_in_vector == -1) EXIT("At line " + std::to_string(counter) + " there is no vehicle id " + tmp[0]);

			if (!check_string_is_double(tmp[1])) EXIT("At line " + std::to_string(counter) + " exit time " + tmp[0] + " is not a double.");
			vehicles[vehicle_index_in_vector].exitTime = std::stod(tmp[1]);
			
			if ((tmp.size() - 2) % 2 == 1) EXIT("At line " + std::to_string(counter) + " expect to have even number of outputs but get " + std::to_string(tmp.size() - 2));
			for (int i = 2 ; i < tmp.size() ; i++) {
				if (!check_string_is_double(tmp[i])) EXIT("At line " + std::to_string(counter) + " each number should be a double, but " + tmp[i] + " is not.");
			}

			for (int j = 2 ; j < tmp.size() ; j = j + 2) {
				double time = std::stod(tmp[j]);
				double acceleration = std::stod(tmp[j + 1]);
				vehicles[vehicle_index_in_vector].times.push_back(time);
				vehicles[vehicle_index_in_vector].accelerations.push_back(acceleration);
			}
		}

		if (counter != num_vehicle) EXIT("Expected to have " + std::to_string(num_vehicle) + " vehicles in output file, but get " + std::to_string(counter));
	}
}


void check_exit_time_bigger_than_arrival_time() {
	for (Vehicle v : vehicles) {
		// TODO
	}
}


void check_acceleration_constraint() {
	for (Vehicle v : vehicles) {
		for (const auto & a : v.accelerations) {
			if (a > MAX_A) EXIT("Vehicle id " + std::to_string(v.id) + " acceleration " + std::to_string(a) + " is over maximum " + std::to_string(MAX_A));
			if (a < MIN_A) EXIT("Vehicle id " + std::to_string(v.id) + " acceleration " + std::to_string(a) + " is under minimum " + std::to_string(MIN_A));
		}
	}
}


void check_velocity_constraint() {
	for (Vehicle v : vehicles) {
		v.velocities.push_back(v.velocity);
		if (v.velocity > MAX_V) EXIT("Vehicle id " + std::to_string(v.id) + " velocity " + std::to_string(v.velocity) + " is over maximum " + std::to_string(MAX_V));
		for (int i = 1 ; i < v.times.size() ; i++) {
			double time_period = v.times[i] - v.times[i - 1];
			v.velocity += time_period * v.accelerations[i - 1];
			v.velocities.push_back(v.velocity);
			if (v.velocity > MAX_V) EXIT("Vehicle id " + std::to_string(v.id) + " velocity " + std::to_string(v.velocity) + " is over maximum " + std::to_string(MAX_V));
		}

		double time_period = v.exitTime - v.times[v.times.size() - 1];
		v.velocity += time_period * v.accelerations[v.times.size() - 1];
		v.velocities.push_back(v.velocity);
		if (v.velocity > MAX_V) EXIT("Vehicle id " + std::to_string(v.id) + " velocity " + std::to_string(v.velocity) + " is over maximum " + std::to_string(MAX_V));
	}	
}



void check_no_overtaking() {
	// TODO
}



void check_safety_time_margin() {
	for (Vehicle v1 : vehicles) {
		for (int i = 1 ; i < v1.times.size() ; i++) {
			double time_start = v1.times[i - 1];
			double time_end = v1.times[i];
			double v = v1.velocities[i - 1];
			double a = v1.accelerations[i - 1];
			// v * t + 1/2 a * t^2 (time_start <= t <= time_end)
			for (Vehicle v2 : vehicles) {
				if (v1.vehicle_id == v2.vehicle_id) continue;
				if (v2.exitTime <= time_start) continue;
				if (v2.earlistArrivalTime >= time_end) continue;
				if (v2.times[0] >= time_start) {
					// TODO
				}
				for (int j = 1 ; j < v2.times.size() ; j++) {
					if (v2.times[j - 1] <= time_start && v2.times[j] >= time_start) {
						double time_start_2 = v2.times[j - 1];
						double time_end_2 = v2.times[j];
						double v_2 = v2.velocities[j - 1];
						double a_2 = v2.accelerations[j - 1];
					}
				}

				for (int k = 1 ; k < v2.times.size() ; k++) {
					if (v2.times[k - 1] <= time_end && v2.times[k] >= time_end) {
						double time_start_3 = v2.times[k - 1];
						double time_end_3 = v2.times[k];
						double v_3 = v2.velocities[k - 1];
						double a_3 = v2.accelerations[k - 1];
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
		for (int i = v.entryIndex ; i < v.exitIndex ; i++) {
			expected_travel_length += section_lengths[i];
		}

		for (int i = 1 ; i < v.times.size() ; i++) {
			double time_period = v.times[i] - v.times[i - 1];
			actual_travel_length += v.velocity * time_period + 0.5 * v.accelerations[i - 1] * time_period * time_period;
			v.velocity += time_period * v.accelerations[i - 1];
		}

		double time_period = v.exitTime - v.times[v.times.size() - 1];
		actual_travel_length += v.velocity * time_period + 0.5 * v.accelerations[v.times.size() - 1] * time_period * time_period;
		if (actual_travel_length != expected_travel_length) EXIT("Vehicle id " + std::to_string(v.id) + " Expect to travel " + std::to_string(expected_travel_length) + " but actual travel " + std::to_string(actual_travel_length));
	}
}


int main(int argc, char* argv[]) {
	if (argc != 3) EXIT("Should have three command line arguments input.");

	std::string input_filename = argv[1];
	std::string output_filename = argv[2];

	check_input_file_is_valid_format(input_filename);
	check_output_file_is_valid_format(output_filename);

	// TODO: exit time should exceed earlist arrival time
	check_exit_time_bigger_than_arrival_time();
	check_acceleration_constraint();
	check_velocity_constraint();
	check_no_overtaking();
	check_safety_time_margin();
	check_finish_the_track();
}