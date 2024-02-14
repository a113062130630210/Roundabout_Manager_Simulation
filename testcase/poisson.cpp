#include <iostream>
#include <random>
#include <cstdlib>
#include <ctime>

#define VEHICLE_MIN 2
#define VEHICLE_MAX 30
#define SECTION_MIN 5
#define SECTION_MAX 20
#define LENGTH_MIN 5
#define LENGTH_MAX 30 
#define MEAN_INTERARRIVAL_TIME 3


int main() {
    srand(time(NULL));
    
    // generate vehicles
    int vehicle_num = rand() % (VEHICLE_MAX - VEHICLE_MIN + 1) + VEHICLE_MIN;
    
    // generate sections
    int section_num = rand() % (SECTION_MAX - SECTION_MIN + 1) + SECTION_MIN;
    int section_length = rand() % (SECTION_MAX - SECTION_MIN + 1) + SECTION_MIN;

    std::cout << vehicle_num << " " << section_num;
    for (int i = 1 ; i <= section_num ; i++) {
        std::cout << " " << section_length;
    }
    std::cout << "\n";

    std::default_random_engine generator;
    std::exponential_distribution<double> distribution(MEAN_INTERARRIVAL_TIME);

    double prev_arrival_time = 0;
    for (int i = 0 ; i < vehicle_num ; i++) {
        double arrival_time_diff = distribution(generator);
        double arrival_time = prev_arrival_time + arrival_time_diff;
        prev_arrival_time = arrival_time;
        // entry section: 0 ~ section_num - 2
        int entry_section = rand() % (section_num - 1);
        // exit section: entry section + 1 ~ section_num - 1
        int exit_section = rand() % (section_num - entry_section - 1) + entry_section + 1;
        std::cout << i << " " << entry_section << " " << exit_section << " " << arrival_time << " " << 0 << "\n";
    }
}
