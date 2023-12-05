all: main

CC = g++
CFLAGS = -std=c++17 -O2 -Wall -Werror -Wextra

main: main.o roundabout_manager.o roundabout.o vehicle.o util.o
	$(CC) -o run.exe main.o roundabout_manager.o roundabout.o vehicle.o util.o
main.o: main.cpp roundabout_manager.hpp roundabout.hpp vehicle.hpp
	$(CC) $(CFLAGS) -c main.cpp
roundabout_manager.o: roundabout_manager.cpp roundabout_manager.hpp roundabout.hpp vehicle.hpp
	$(CC) $(CFLAGS) -c roundabout_manager.cpp
roundabout.o: roundabout.cpp roundabout.hpp vehicle.hpp
	$(CC) $(CFLAGS) -c roundabout.cpp
vehicle.o: vehicle.cpp vehicle.hpp constants.hpp util.hpp
	$(CC) $(CFLAGS) -c vehicle.cpp
util.o: util.cpp util.hpp
	$(CC) $(CFLAGS) -c util.cpp
