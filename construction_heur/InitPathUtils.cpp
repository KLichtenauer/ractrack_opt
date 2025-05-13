//
// Created by Privati on 23.04.25.
//

#include "InitPathUtils.h"

#include <deque>
#include <iomanip>

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

    exportClearanceMap(clearance, t, "clearance.t");



    using State = pair<int, Coord>;

    vector<vector<State> > weightedGrid(t.height(), vector<State>(t.width(), {INT_MAX, Coord(0, 0)}));
    for (int c = 0; c < t.width(); c++) {
        for (int r = 0; r < t.height(); r++) {
            weightedGrid[r][c].second = Coord{r, c};
        }
    }
    const Coord start = t.start;
    weightedGrid[start.row][start.col].first = 0;
    deque<Coord> qPath;
    qPath.push_back(start);
    while (!qPath.empty() && qPath.size() > 0) {
        Coord c = qPath.front();
        qPath.pop_front();
        for (auto [x, y]: {
                 pair<int, int>{c.row - 1, c.col},
                 pair<int, int>{c.row, c.col - 1},
                 pair<int, int>{c.row + 1, c.col},
                 pair<int, int>{c.row, c.col + 1}
             }) {

            int K = 2;
            int EXTRA_GRASS_COST = 4;
            int base = 1;

            if (x < 0 || y < 0 || x >= t.height() || y >= t.width()) continue;
            if (t.at(x, y) == 'O' || t.at(x, y) == 'S') continue;
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

    exportWeightedGrid(weightedGrid, t, "weighted_grid.t");



    vector<Coord> path;
    vector<Coord> finishLine = t.finishLine;
    Coord bestFinishTile = finishLine.at(0);
    for (auto finishTile: finishLine) {
        if (weightedGrid[finishTile.row][finishTile.col].first < weightedGrid[bestFinishTile.row][bestFinishTile.col].first) {
            bestFinishTile = Coord(finishTile);
        }

    }
    bool arrivedStart = false;
    Coord currentCoord = bestFinishTile;
    path.push_back(bestFinishTile);

    while (!arrivedStart) {
        Coord parent = weightedGrid[currentCoord.row][currentCoord.col].second;
        path.insert(path.begin(), parent);
        if (weightedGrid[parent.row][parent.col].first == 0 || (parent.row == start.row && parent.col == start.col)) {
            arrivedStart = true;
        }
        currentCoord = parent;
    }


    return path;
}

void InitPathUtils::exportClearanceMap(const vector<vector<int>>& clearance, const Track& t, const string& filename) {
    ofstream out(filename);
    for (int r = 0; r < t.height(); ++r) {
        for (int c = 0; c < t.width(); ++c) {
            if (t.at(r, c) == 'G') out << " G ";
            else if (t.at(r, c) == 'O') out << " O ";
            else if (clearance[r][c] == INT_MAX) out << " . ";
            else out << setw(2) << clearance[r][c] << " ";
        }
        out << "\n";
    }
    out.close();
}

void InitPathUtils::exportWeightedGrid(const vector<vector<pair<int, Coord>>>& grid, const Track& t, const string& filename) {
    ofstream out(filename);
    for (int r = 0; r < t.height(); ++r) {
        for (int c = 0; c < t.width(); ++c) {
            if (t.at(r, c) == 'O') out << " O ";
            else if (t.at(r, c) == 'G') out << " G ";
            else if (grid[r][c].first == INT_MAX) out << " . ";
            else out << setw(2) << grid[r][c].first << " ";
        }
        out << "\n";
    }
    out.close();
}




