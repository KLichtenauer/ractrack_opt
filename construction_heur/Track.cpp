//
// Created by Privati on 23.04.25.
//

#include "Track.h"

#include <vector>

#include "InitPathUtils.h"


string Track::to_json(const vector<State>& path, const int height) {

    // Add points on track:
    string text = "";
    for (auto points : path) {
        const int invertedHeight = height - (points.pos.row + 1);
        text += to_string(points.pos.col) + ", " + to_string(invertedHeight) + "\n";
    }

    return text;
}
