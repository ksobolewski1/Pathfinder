#pragma once

#include <SFML/Graphics.hpp>

#include <iostream>
#include <vector> 
#include <unordered_map> // for A* to check if nodes have been visited before or have been enqueued
#include <tuple> 
#include <chrono> // for interface cooldown 
#include <random> // random point seeding for triangulation 
#include <cmath> // sqrt
#include <chrono> // timing the search algorithms 
