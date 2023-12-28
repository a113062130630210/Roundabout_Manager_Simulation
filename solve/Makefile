all: main

CC = g++
CFLAGS = -std=c++17 -O2 -Wall -Werror -Wextra

main: build build/main.o build/roundabout_manager.o build/roundabout.o build/vehicle.o build/util.o
	cd build; \
	$(CC) -o run.exe main.o roundabout_manager.o roundabout.o vehicle.o util.o

build: 
	mkdir build
build/main.o: main.cpp roundabout_manager.hpp roundabout.hpp vehicle.hpp
	$(CC) $(CFLAGS) -c main.cpp -o build/main.o
build/roundabout_manager.o: roundabout_manager.cpp roundabout_manager.hpp roundabout.hpp vehicle.hpp
	$(CC) $(CFLAGS) -c roundabout_manager.cpp -o build/roundabout_manager.o
build/roundabout.o: roundabout.cpp roundabout.hpp vehicle.hpp
	$(CC) $(CFLAGS) -c roundabout.cpp -o build/roundabout.o
build/vehicle.o: vehicle.cpp vehicle.hpp constants.hpp util.hpp
	$(CC) $(CFLAGS) -c vehicle.cpp -o build/vehicle.o
build/util.o: util.cpp util.hpp
	$(CC) $(CFLAGS) -c util.cpp -o build/util.o
