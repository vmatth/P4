#pragma once
#include <list>

struct Position{
    double x = 0;
    double y = 0;
};


struct AStarPathInfo{
    Position cellPos;
    std::list<Position> path;
};
