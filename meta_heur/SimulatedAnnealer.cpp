#include "SimulatedAnnealer.h"
#include <algorithm>
#include <queue>
#include <unordered_map>
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
    lastOp = std::uniform_int_distribution<int>{0,2}(rng);
    switch (lastOp) {
        case 0: tryTwoStepShortcut(p);    break;
        case 1: tryThreeStepShortcut(p);    break;
        //case 2: tryInsertStep(p) break; NOT WORKING AS EXPECTED
    }

    return p;
}

// if p[i]→p[i+2] is legal, delete p[i+1]
void SimulatedAnnealer::tryTwoStepShortcut(vector<State>& p) {
    if (p.size() < 3) return;
    int i = uniform_int_distribution<int>{0,int(p.size())-3}(rng);

    State &A = p[i], &B = p[i+1], &C = p[i+2];
    Coord jump = { C.pos.row - A.pos.row, C.pos.col - A.pos.col };
    if (abs(jump.row - A.vel.row) > 1 || abs(jump.col - A.vel.col) > 1) return;
    if ((A.vel.row>0 && jump.row<0) ||
        (A.vel.row<0 && jump.row>0) ||
        (A.vel.col>0 && jump.col<0) ||
        (A.vel.col<0 && jump.col>0)) return;
    if (!isPathClear(A.pos, C.pos)) return;
    if (t.at(A.pos.row, A.pos.col)=='G' && violatesGrassRule(A.vel, jump)) return;

    State Bcopy = B; State Ccopy = C;
    p.erase(p.begin() + i + 1);
    p[i + 1].vel = jump;

    if (i+2 < p.size()) {
        const State& from   = p[i+1];
        const Coord& target = p[i+2].pos;
        Coord neededVel = { target.row - from.pos.row, target.col - from.pos.col };

        bool ok = true;
        if (!legalJump(from.vel, neededVel)) {ok = false;}
        else if (!isPathClear(from.pos, target)) {ok = false;}
        else if (t.at(from.pos.row, from.pos.col)=='G' && violatesGrassRule(from.vel, neededVel)) {ok = false;}

        if (!ok) {
            p.insert(p.begin() + i + 1, Bcopy);
            p[i + 2] = Ccopy;
        }
    }
}

// if p[i]→p[i+3] is legal, delete p[i+1] and p[i+2]
void SimulatedAnnealer::tryThreeStepShortcut(std::vector<State>& p) {
    if (p.size() < 4) return;
    int i = std::uniform_int_distribution<int>{0, int(p.size()) - 4}(rng);
    State &A = p[i], &B = p[i + 1], &C = p[i + 2], &D = p[i + 3];
    Coord jump = { D.pos.row - A.pos.row,
                   D.pos.col - A.pos.col };

    if (std::abs(jump.row - A.vel.row) > 1 || std::abs(jump.col - A.vel.col) > 1) return;

    if ((A.vel.row > 0 && jump.row < 0) ||
        (A.vel.row < 0 && jump.row > 0) ||
        (A.vel.col > 0 && jump.col < 0) ||
        (A.vel.col < 0 && jump.col > 0))
        return;

    if (!isPathClear(A.pos, D.pos)) return;
    if (t.at(A.pos.row, A.pos.col) == 'G' && violatesGrassRule(A.vel, jump)) return;

    State Bcopy = B, Ccopy = C, Dcopy = D;

    p.erase(p.begin() + i + 1, p.begin() + i + 3);
    p[i + 1].vel = jump;

    if (i + 2 < p.size()) {
        const State& from   = p[i + 1];
        const Coord& target = p[i + 2].pos;
        Coord neededVel = { target.row - from.pos.row, target.col - from.pos.col };

        bool ok = true;
        if (!legalJump(from.vel, neededVel)) {ok = false;}
        else if (!isPathClear(from.pos, target)) {ok = false;}
        else if (t.at(from.pos.row, from.pos.col) == 'G' && violatesGrassRule(from.vel, neededVel)) {ok = false;}

        if (!ok) {
            p.insert(p.begin() + i + 1, { Bcopy, Ccopy });
            p[i + 3] = Dcopy;
        }
    }
}


void SimulatedAnnealer::tryInsertStep(std::vector<State>& p) {
    if (p.size() < 2) return;

    int i = std::uniform_int_distribution<int>{0, int(p.size()) - 2}(rng);
    State &A = p[i], &B = p[i+1];

    std::vector<Coord> deltas = {
        { -1,-1 },{ -1,0 },{ -1,1 },
        {  0,-1 },{  0,0 },{  0,1 },
        {  1,-1 },{  1,0 },{  1,1 }
    };
    std::shuffle(deltas.begin(), deltas.end(), rng);

    for (auto d : deltas) {
        Coord vel = { A.vel.row + d.row, A.vel.col + d.col };
        Coord insertPos = { A.pos.row + vel.row, A.pos.col + vel.col };

        if (!legalJump(A.vel, vel)) continue;
        if (!isInside(insertPos)) continue;
        if (!isPathClear(A.pos, insertPos)) continue;
        if (t.at(A.pos.row, A.pos.col)=='G' && violatesGrassRule(A.vel, vel)) continue;
        if (std::any_of(p.begin(), p.end(), [&](auto &s){return s.pos.col == insertPos.col && s.pos.row == insertPos.row;})) continue;

        Coord jumpToNext = { B.pos.row - insertPos.row,B.pos.col - insertPos.col };
        if (!legalJump(vel, jumpToNext)) continue;
        if (!isPathClear(insertPos, B.pos)) continue;
        if (t.at(insertPos.row, insertPos.col)=='G' && violatesGrassRule(vel, jumpToNext)) continue;

        State inserted{insertPos, vel};
        p.insert(p.begin() + i + 1, inserted);
        return;
    }
}

std::vector<State> SimulatedAnnealer::run(double startTemp, double coolingRate, double minTemp, int maxIters) {
    auto best = currentPath;
    double bestCost = computeCost(currentPath);
    double temp = startTemp;

    while (temp > minTemp) {
        auto candidate = mutatePath(currentPath);
        double currentCost = computeCost(currentPath);
        double candidateCost = computeCost(candidate);

        double delta = candidateCost - currentCost;
        if (lastOp == 1 || delta < 0 || uni(rng) < std::exp(-delta / temp)) {
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
            << "  insert:   " << opCounts[1] << "\n"
            << "  three step:   " << opCounts[2] << "\n";
    return best;
}