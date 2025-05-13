//
// Created by Privati on 23.04.25.
//

#ifndef INITPATH_H
#define INITPATH_H
#include <vector>

#include "Track.h"


class InitPathUtils {
public:
    static vector<Coord> initPath(Track& t);

    static void exportClearanceMap(const vector<vector<int>> &clearance, const Track &t, const string &filename);

    static void exportWeightedGrid(const vector<vector<pair<int, Coord>>> &grid, const Track &t, const string &filename);
};



#endif //INITPATH_H
