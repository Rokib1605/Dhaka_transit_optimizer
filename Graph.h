#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <string>
#include <cmath>
#include <map>

struct LatLong {
    double lat, lon;
    // Map key comparison
    bool operator<(const LatLong& other) const {
        return (lat < other.lat) || (lat == other.lat && lon < other.lon);
    }
};

enum Mode { CAR, METRO, BIKOLPO_BUS, UTTARA_BUS, WALK };

struct Edge {
    LatLong to;
    double distance; 
    Mode mode;
    double speed;    
    double costPerKm; 
};

struct Node {
    LatLong pos;
    std::vector<Edge> neighbors;
};

const double CAR_SPEED = 20.0, METRO_SPEED = 15.0, BIKOLPO_SPEED = 10.0, UTTARA_SPEED = 12.0, WALK_SPEED = 2.0;
const double CAR_COST = 20.0, METRO_COST = 5.0, BIKOLPO_COST = 7.0, UTTARA_COST = 10.0, WALK_COST = 0.0;

#endif