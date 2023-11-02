#include <iostream>
#include <vector>
#include "simulate.h"

using namespace std;

int main() {
	int numVehicle, numSection;
	double maxAllowedVelocity;
	vector<Vehicle> vehicles;
	cin >> numVehicle >> numSection >> maxAllowedVelocity;
	for (int i = 1 ; i <= numVehicle ; i++) {
		int entryIndex, exitIndex;
		double earlistArrivalTime;
		cin >> entryIndex >> exitIndex >> earlistArrivalTime;
		Vehicle vehicle(i, entryIndex, exitIndex, earlistArrivalTime);
		vehicles.push_back(vehicle);
	}
	RoundaboutManager manager(numVehicle, numSection, maxAllowedVelocity, vehicles);
	cout << manager.solve();
}