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

    
struct SubAreaInterval{
    int startX = 0;
    int endX = 0;

    int startY = 0;
    int endY = 0;
};
