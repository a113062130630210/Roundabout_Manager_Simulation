all: main

CXX = g++
CPPFLAGS = -std=c++20 -O2 -Wall -Werror -Wextra

main: build build/main.o build/roundabout_manager.o build/roundabout.o build/vehicle.o build/trajectory.o build/modular.o build/util.o
	cd build; \
	$(CXX) -o run.exe main.o roundabout_manager.o roundabout.o vehicle.o trajectory.o modular.o util.o

build: 
	mkdir build
build/main.o: main.cpp roundabout_manager.hpp roundabout.hpp vehicle.hpp
	$(CXX) $(CPPFLAGS) -c main.cpp -o build/main.o
build/roundabout_manager.o: roundabout_manager.cpp roundabout_manager.hpp roundabout.hpp vehicle.hpp
	$(CXX) $(CPPFLAGS) -c roundabout_manager.cpp -o build/roundabout_manager.o
build/roundabout.o: roundabout.cpp roundabout.hpp vehicle.hpp
	$(CXX) $(CPPFLAGS) -c roundabout.cpp -o build/roundabout.o
build/vehicle.o: vehicle.cpp vehicle.hpp trajectory.hpp modular.hpp
	$(CXX) $(CPPFLAGS) -c vehicle.cpp -o build/vehicle.o
build/trajectory.o: trajectory.cpp trajectory.hpp vehicle.hpp constants.hpp util.hpp
	$(CXX) $(CPPFLAGS) -c trajectory.cpp -o build/trajectory.o
build/modular.o: modular.hpp
	$(CXX) $(CPPFLAGS) -x c++ -c modular.hpp -o build/modular.o
build/util.o: util.cpp util.hpp
	$(CXX) $(CPPFLAGS) -c util.cpp -o build/util.o

clear:
	rm build/*