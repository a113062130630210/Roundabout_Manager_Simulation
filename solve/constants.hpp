#pragma once

#include <iostream>

// should be a negative number
#define MIN_A -3.

// should be a positive number
#define MAX_A 3.

// should be a positive number
#define MAX_V 15.

// should be a positive number
#define TIME_GAP 1.5

// should be a positive number
#define MEAN_INTERARRIVAL_TIME 10

#define EXIT(s) do { std::cerr << s << std::endl; exit(1); } while (0)
