//
// Created by vollm on 5/27/2025.
//

#ifndef STATE_H
#define STATE_H

class Coord {
public:
    int row;
    int col;

    Coord(int r, int c)
      : row(r),
        col(c) {
    }
};


struct State {
    Coord pos;
    Coord vel;
    State() : pos(0,0), vel(0,0) {}
    State(const Coord &p, const Coord &v) : pos(p), vel(v) {}
};

#endif //STATE_H
