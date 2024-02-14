#!/bin/bash
num_testcases=30

g++ -o poisson poisson.cpp

if [ $? -eq 0 ]; then
    for ((i=0; i<$num_testcases; i++)); do
        ./poisson > "$i.in"
    done
fi
