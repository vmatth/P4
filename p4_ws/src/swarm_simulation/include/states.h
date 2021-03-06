#pragma once


enum State{Wall, Unexplored, Free, TempWall};

enum MovementState{idle, direct, turning, wall};

//Specifies the type of movement that the turtlebot will perform
//Turn: Turns in place
//Traverse: Turns and moves towards a point
enum MovementType{turn, traverse};

enum TurnType{relative, absolute};