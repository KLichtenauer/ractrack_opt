//
// Created by Privati on 23.04.25.
//

#include "Track.h"

#include <vector>


string Track::to_json(const vector<Coord>& path, const int height) {

    // Add points on track:
    string text = "";
    for (auto points : path) {
        const int invertedHeight = height - (points.row + 1);
        text += to_string(points.col) + ", " + to_string(invertedHeight) + "\n";
    }

    return text;
}
