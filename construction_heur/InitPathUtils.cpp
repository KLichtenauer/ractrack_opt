//
// Created by Privati on 23.04.25.
//

#include "InitPathUtils.h"
#include "Track.h"

vector<Coord> InitPathUtils::initPath(Track &t) {

    // Clearance maps each tile to a number defining how far its away from grass.
    deque<Coord> qClearance;
    vector<vector<int> > clearance(t.height(), vector<int>(t.width(), INT_MAX));

    // Init clearance with Grass map.
    for (int row = 0; row < t.height(); row++) {
        for (int column = 0; column < t.width(); column++) {
            if (t.at(row, column) == 'G') {
                clearance[row][column] = 0;
                qClearance.emplace_back(row, column);
            }
        }
    }

    cout << "\n Starting clearance evaluation.";

    while (!qClearance.empty() && qClearance.size() > 0) {
        Coord c = qClearance.front();
        qClearance.pop_front();
        for (auto [row, column]: {
                 pair<int, int>{c.row - 1, c.col},
                 pair<int, int>{c.row, c.col - 1},
                 pair<int, int>{c.row + 1, c.col},
                 pair<int, int>{c.row, c.col + 1}
             }) {
            if (row < 0 || column < 0 || row >= t.height() || column >= t.width()) continue;
            if (t.at(row, column) == 'O' || t.at(row, column) == 'G') continue;

            if (clearance[row][column] > clearance[c.row][c.col] + 1) {
                clearance[row][column] = clearance[c.row][c.col] + 1;
                qClearance.emplace_back(row, column);
            }
        }
    }

    cout << "\n Finished clearance.";


    using State = pair<int, Coord>;

    vector<vector<State> > weightedGrid(t.height(), vector<State>(t.width(), {INT_MAX, Coord(0, 0)}));

    for (int c = 0; c < t.width(); c++) {
        for (int r = 0; r < t.height(); r++) {
            weightedGrid[r][c].second = Coord{r, c};
        }
    }


    // Plan the shortest path with weighted map to avoid grass.
    const Coord start = t.start;
    weightedGrid[start.row][start.col].first = 0;

    deque<Coord> qPath;
    qPath.push_back(start);

    while (!qPath.empty() && qPath.size() > 0) {
        Coord c = qPath.front();
        qPath.pop_front();

        // Only moves are up, down, left, right (nothing diagonal)
        for (auto [x, y]: {
                 pair<int, int>{c.row - 1, c.col},
                 pair<int, int>{c.row, c.col - 1},
                 pair<int, int>{c.row + 1, c.col},
                 pair<int, int>{c.row, c.col + 1}
             }) {

            int K = 2;
            int EXTRA_GRASS_COST = 4; // Costs for being in grass. (If move takes 5 normal tiles it chooses the grass path rather than the way around)
            int base = 1; // Every move costs one

            if (x < 0 || y < 0 || x >= t.height() || y >= t.width()) continue;
            // Continue if Finish, Obstacle or Start tile
            if (t.at(x, y) == 'O' || t.at(x, y) == 'S') continue;


            // Else:
            const int currentCost = weightedGrid[c.row][c.col].first + K / (clearance[x][y] + 1) + base +
                              (t.at(x, y) == 'G'
                                   ? EXTRA_GRASS_COST
                                   : 0);

            if (weightedGrid[x][y].first > currentCost) {
                weightedGrid[x][y].first = currentCost;
                weightedGrid[x][y].second = Coord{c.row, c.col};

                if ( t.at(x, y) == 'F') {
                    continue;
                }
                qPath.emplace_back(x, y);
            }
        }
    }

    cout << "Finished planing shortest path. \n";

    // Extracting path. The final path:
    vector<Coord> path;

    // Find the best solution from the finish line entries.
    vector<Coord> finishLine = t.finishLine;
    Coord bestFinishTile = finishLine.at(0);
    for (auto finishTile: finishLine) {
        if (weightedGrid[finishTile.row][finishTile.col].first < weightedGrid[bestFinishTile.row][bestFinishTile.col].first) {
            bestFinishTile = Coord(finishTile);
        }

    }
    // Track back with parents and put it in paths.
    bool arrivedStart = false;
    Coord currentCoord = bestFinishTile;
    path.push_back(bestFinishTile);

    while (!arrivedStart) {
        Coord parent = weightedGrid[currentCoord.row][currentCoord.col].second;
        path.insert(path.begin(), parent);
        // Stop when distance of point is 0 (starting point found).
        if (weightedGrid[parent.row][parent.col].first == 0 || (parent.row == start.row && parent.col == start.col)) {
            cout << "\n Start found" << parent.row << "," << parent.col;
            arrivedStart = true;
        }
        currentCoord = parent;
    }


    cout << "\n Minimum path found with size: " + to_string(path.size());


    return path;
}


