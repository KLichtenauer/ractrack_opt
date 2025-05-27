#include "SimulatedAnnealer.h"
#include <algorithm>
#include <unordered_set>
#include <../construction_heur/Track.h>
#include <../construction_heur/InitPathUtils.h>

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
        case 0: tryMultiShortcut(p);    break;
        case 1: trySingleRemoval(p);    break;
        case 2: tryRandomInsertion(p);  break;
    }

    return p;
}

// --- operator #1: multi-node shortcut ---
void SimulatedAnnealer::tryMultiShortcut(std::vector<State>& p) {
    if (p.size() < 3) return;
    // pick i < j at least 2 apart
    std::uniform_int_distribution<int> distI(0, p.size()-3);
    int i = distI(rng);
    std::uniform_int_distribution<int> distJ(i+2, p.size()-1);
    int j = distJ(rng);

    const State &si = p[i], &sj = p[j];
    // compute implied velocity vector
    Coord delta = { sj.pos.row - si.pos.row,
                    sj.pos.col - si.pos.col };
    // implied newVel must obey your vel limits, and accel from si.vel
    if (std::abs(delta.row - si.vel.row) > 1 ||
        std::abs(delta.col - si.vel.col) > 1)
        return;
    // line‐of‐sight check
    if (!isPathClear(si.pos, sj.pos)) return;

    // splice out [i+1 .. j-1]
    p.erase(p.begin() + i + 1, p.begin() + j);
    // adjust velocity in the new jump‐in state
    p[i+1].vel = delta;
}

// --- operator #2: single-node removal ---
void SimulatedAnnealer::trySingleRemoval(std::vector<State>& p) {
    if (p.size() < 4) return;
    std::uniform_int_distribution<int> dist(1, p.size()-2);
    int i = dist(rng);
    const State &prev = p[i-1], &next = p[i+1];
    // can we go from prev to next directly?
    Coord delta = { next.pos.row - prev.pos.row,
                    next.pos.col - prev.pos.col };
    if (std::abs(delta.row - prev.vel.row) > 1 ||
        std::abs(delta.col - prev.vel.col) > 1) return;
    if (!isPathClear(prev.pos, next.pos)) return;

    // remove p[i]
    p.erase(p.begin() + i);
    // fix new velocity at index i
    p[i].vel = delta;
}

// --- operator #3: random insertion ---
void SimulatedAnnealer::tryRandomInsertion(std::vector<State>& p) {
    if (p.empty()) return;
    std::uniform_int_distribution<int> dist(0, p.size()-1);
    int i = dist(rng);

    // random accel each axis in {-1,0,1}
    int ax = std::uniform_int_distribution<int>{-1,1}(rng);
    int ay = std::uniform_int_distribution<int>{-1,1}(rng);

    State s = p[i];
    Coord newVel = { s.vel.row + ax,
                     s.vel.col + ay };
    Coord newPos = { s.pos.row + newVel.row,
                     s.pos.col + newVel.col };

    // bounds + obstacle check
    if (!isInside(newPos) || t.at(newPos.row,newPos.col) == 'O') return;
    // clear path check
    if (!isPathClear(s.pos, newPos)) return;

    // build new state and insert
    State ns;
    ns.pos = newPos;
    ns.vel   = newVel;
    p.insert(p.begin() + i + 1, ns);
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
            << "  multi shortcut:   " << opCounts[0] << "\n"
            << "  single removal:   " << opCounts[1] << "\n"
            << "  random insert:    " << opCounts[2] << "\n";
    return best;
}