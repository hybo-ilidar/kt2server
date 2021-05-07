#!/bin/bash
g++ -std=c++11 main.cpp sensor.cpp -pthread -lserialport -o kt2server
