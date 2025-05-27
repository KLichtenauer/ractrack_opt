#include "SimulatedAnnealer.h"
#include <algorithm>
#include <unordered_set>
#include <../construction_heur/Track.h>
#include <../construction_heur/InitPathUtils.h>

static bool violatesGrassRule(const Coord &oldVel, const Coord &newVel) {
    int fromX = std::abs(oldVel.row), toX = std::abs(newVel.row);
    int fromY = std::abs(oldVel.col), toY = std::abs(newVel.col);

    bool badX = (fromX >= 2 && toX >= fromX)
             || (fromX == 1 && toX >  fromX);
    bool badY = (fromY >= 2 && toY >= fromY)
             || (fromY == 1 && toY >  fromY);

    return badX || badY;
}

static bool legalJump(const Coord &oldVel, const Coord &newVel) {
    if (std::abs(newVel.row - oldVel.row) > 1 ||
        std::abs(newVel.col - oldVel.col) > 1)
        return false;
    if (oldVel.row  * newVel.row < 0 ||
        oldVel.col  * newVel.col < 0)
        return false;

    return true;
}

static bool legalJump(const State &s, const Coord &jump) {
    return legalJump(s.vel, jump);
}


SimulatedAnnealer::SimulatedAnnealer(const Track& track, std::vector<State> initialPath)
    : t(track), currentPath(std::move(initialPath)), rng(std::random_device{}()), uni(0.0, 1.0) {}

bool SimulatedAnnealer::isInside(const Coord& c) const {
    return c.row >= 0 && c.col >= 0 && c.row < t.height() && c.col < t.width();
}

bool SimulatedAnnealer::isPathClear(const Coord& from, const Coord& to) const {
    int x1 = from.row, y1 = from.col;
    int x2 = to.row, y2 = to.col;
    int dx = std::abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
    int dy = -std::abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (!isInside({x1, y1}) || t.at(x1, y1) == 'O') return false;
        if (x1 == x2 && y1 == y2) break;

        int oldX = x1, oldY = y1;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x1 += sx; }
        if (e2 <= dx) { err += dx; y1 += sy; }

        if (x1 != oldX && y1 != oldY) {
            if (t.at(oldX, y1) == 'O' || t.at(x1, oldY) == 'O') return false;
        }
    }
    return true;
}

double SimulatedAnnealer::computeCost(const std::vector<State>& path) {
    return static_cast<double>(path.size());
}

std::vector<State> SimulatedAnnealer::mutatePath(const std::vector<State>& path) {
    std::vector<State> p = path;
    lastOp = std::uniform_int_distribution<int>{0,1}(rng);
    switch (lastOp) {
        case 0: tryTwoStepShortcut(p);    break;
        case 1: tryThreeStepShortcut(p);    break;
    }

    return p;
}

// if p[i]→p[i+2] is legal, delete p[i+1]
void SimulatedAnnealer::tryTwoStepShortcut(vector<State>& p) {
    if (p.size() < 3) return;
    int i = uniform_int_distribution<int>{0,int(p.size())-3}(rng);

    State &A = p[i], &B = p[i+1], &C = p[i+2];
    Coord jump = { C.pos.row - A.pos.row,
                   C.pos.col - A.pos.col };

    // 1) accel ≤1 per axis
    if (abs(jump.row - A.vel.row) > 1 ||
        abs(jump.col - A.vel.col) > 1) return;

    // 2) no reversal
    if ((A.vel.row>0 && jump.row<0) ||
        (A.vel.row<0 && jump.row>0) ||
        (A.vel.col>0 && jump.col<0) ||
        (A.vel.col<0 && jump.col>0)) return;

    // 3) line‐of‐sight
    if (!isPathClear(A.pos, C.pos)) return;

    // 4) grass‐brake at A if needed
    if (t.at(A.pos.row, A.pos.col)=='G' && violatesGrassRule(A.vel, jump))
        return;

    // 5) commit
    p.erase(p.begin() + i + 1);
    p[i+1].vel = jump;
}

void SimulatedAnnealer::tryThreeStepShortcut(vector<State>& p) {
    if (p.size() < 4) return;
    int i = uniform_int_distribution<int>{0,int(p.size())-4}(rng);

    State &A = p[i], &D = p[i+3];
    Coord jump = { D.pos.row - A.pos.row,
                   D.pos.col - A.pos.col };

    if (!legalJump(A, jump) || !isPathClear(A.pos, D.pos))
        return;

    p.erase(p.begin()+i+1, p.begin()+i+3);
    p[i+1].vel = jump;
}


std::vector<State> SimulatedAnnealer::run(double startTemp, double coolingRate, double minTemp, int maxIters) {
    auto best = currentPath;
    double bestCost = computeCost(currentPath);
    double temp = startTemp;

    for (int iter = 0; iter < maxIters && temp > minTemp; ++iter) {
        auto candidate = mutatePath(currentPath);
        double currentCost = computeCost(currentPath);
        double candidateCost = computeCost(candidate);

        double delta = candidateCost - currentCost;
        if (delta < 0 || uni(rng) < std::exp(-delta / temp)) {
            currentPath = candidate;
            if (candidateCost < bestCost) {
                best = candidate;
                bestCost = candidateCost;
                opCounts[lastOp]++;
            }
        }

        temp *= coolingRate;
    }

    std::cout << "Operator usage:\n"
            << "  two step:   " << opCounts[0] << "\n"
            << "  three step:   " << opCounts[1] << "\n";
    return best;
}