//
// Created by Privati on 23.04.25.
//

#include "Track.h"


string Track::to_json(const vector<Coord>& path) {

    // Add points on track:
    string text = "";
    for (auto points : path) {
        text += to_string(points.row) + ", " + to_string(points.col) + "\n";
    }

    return text;
}
