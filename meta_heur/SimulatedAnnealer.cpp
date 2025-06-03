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
    lastOp = std::uniform_int_distribution<int>{0,4}(rng);
    switch (lastOp) {
        case 0: tryTwoStepShortcut(p);    break;
        case 1: tryThreeStepShortcut(p);    break;
        //case 2: tryInsertStep(p) break; NOT WORKING AS EXPECTED
        case 2: tryAccelDecelStraight(p); break;
        case 3: tryLongShortcut(p); break;
        case 4: tryDiagonalCorner(p); break;
    }

    return p;
}

// if p[i]→p[i+2] is legal, delete p[i+1]
void SimulatedAnnealer::tryTwoStepShortcut(std::vector<State>& p) {
    if (p.size() < 3) return;
    int i = std::uniform_int_distribution<int>{0, int(p.size()) - 3}(rng);

    State &A = p[i], &B = p[i + 1], &C = p[i + 2];
    Coord jump{ C.pos.row - A.pos.row, C.pos.col - A.pos.col };

    // Check direct jump A -> C
    if (!legalJump(A.vel, jump)) return;
    if (!isPathClear(A.pos, C.pos)) return;
    if (t.at(A.pos.row, A.pos.col) == 'G' && violatesGrassRule(A.vel, jump)) return;

    // Backup middle node (B)
    std::vector<State> backup{B};

    // Simulate new state at C
    State newC = C;
    newC.vel = jump;

    // Check A' -> A
    bool ok = true;
    if (i - 1 >= 0) {
        const State& prev = p[i - 1];
        Coord toA{ A.pos.row - prev.pos.row, A.pos.col - prev.pos.col };
        if (!legalJump(prev.vel, toA) || !isPathClear(prev.pos, A.pos))
            ok = false;
        if (t.at(prev.pos.row, prev.pos.col) == 'G' && violatesGrassRule(prev.vel, toA))
            ok = false;
    }

    // Check newC -> p[i+3] if it exists
    if (ok && i + 3 < (int)p.size()) {
        const Coord& nextPos = p[i + 3].pos;
        Coord toNext{ nextPos.row - C.pos.row, nextPos.col - C.pos.col };
        if (!legalJump(jump, toNext) || !isPathClear(C.pos, nextPos))
            ok = false;
        if (t.at(C.pos.row, C.pos.col) == 'G' && violatesGrassRule(jump, toNext))
            ok = false;
    }

    if (!ok) return;

    // Commit change
    p.erase(p.begin() + i + 1);       // remove B
    p[i + 1].vel = jump;              // new velocity at C
}

void SimulatedAnnealer::tryDiagonalCorner(std::vector<State>& p)
{
    if (p.size() < 3) return;
    int i = std::uniform_int_distribution<int>{1, (int)p.size()-2}(rng);

    const State& A = p[i-1];
    const State& B = p[i];
    const State& C = p[i+1];

    Coord d1{B.pos.row-A.pos.row, B.pos.col-A.pos.col};
    Coord d2{C.pos.row-B.pos.row, C.pos.col-B.pos.col};
    if ( (d1.row==0 && d2.col==0) || (d1.col==0 && d2.row==0) ) {
        Coord diag{ d1.row+d2.row, d1.col+d2.col };
        Coord midPos{ A.pos.row+diag.row, A.pos.col+diag.col };

        if ( isInside(midPos)
          && isPathClear(A.pos, midPos)
          && isPathClear(midPos, C.pos) )
        {
            // legal?  check velocity constraints
            if ( legalJump(A.vel, diag)
              && legalJump(diag, C.vel)
              && !( t.at(A.pos.row,A.pos.col)=='G'
                    && violatesGrassRule(A.vel, diag) ) )
            {
                State S{ midPos, diag };
                p[i] = S;                 // overwrite B by diagonal step
            }
        }
    }
}


// if p[i]→p[i+3] is legal, delete p[i+1] and p[i+2]
void SimulatedAnnealer::tryThreeStepShortcut(std::vector<State>& p) {
    if (p.size() < 4) return;

    int i = std::uniform_int_distribution<int>{0, (int)p.size() - 4}(rng);

    State& A = p[i];
    State& B = p[i + 1];
    State& C = p[i + 2];
    State& D = p[i + 3];

    Coord jump{ D.pos.row - A.pos.row, D.pos.col - A.pos.col };

    if (!legalJump(A.vel, jump)) return;
    if (!isPathClear(A.pos, D.pos)) return;
    if (t.at(A.pos.row, A.pos.col) == 'G' && violatesGrassRule(A.vel, jump)) return;

    std::vector<State> backup{ B, C };

    State newD = D;
    newD.vel = jump;

    bool ok = true;
    if (i - 1 >= 0) {
        const State& prev = p[i - 1];
        Coord toA{ A.pos.row - prev.pos.row, A.pos.col - prev.pos.col };

        if (!legalJump(prev.vel, toA) || !isPathClear(prev.pos, A.pos))
            ok = false;
        if (t.at(prev.pos.row, prev.pos.col) == 'G' &&
            violatesGrassRule(prev.vel, toA))
            ok = false;
    }

    if (ok && i + 4 < (int)p.size()) {
        const State& E = p[i + 4];
        Coord toE{ E.pos.row - D.pos.row, E.pos.col - D.pos.col };

        if (!legalJump(jump, toE) || !isPathClear(D.pos, E.pos))
            ok = false;
        if (t.at(D.pos.row, D.pos.col) == 'G' &&
            violatesGrassRule(jump, toE))
            ok = false;
    }

    if (ok) {
        p.erase(p.begin() + i + 1, p.begin() + i + 3); // remove B, C
        p[i + 1].vel = jump;                           // update D
    } else {
        // rollback: leave p unchanged
        // (do nothing — shortcut is aborted)
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

void SimulatedAnnealer::tryLongShortcut(std::vector<State>& p)
{
    if (p.size() < 4) return;

    int i = std::uniform_int_distribution<int>{0, (int)p.size() - 4}(rng);
    int maxSkip = std::min(10, (int)p.size() - i - 2);

    for (int k = maxSkip; k >= 2; --k) {
        State &A = p[i];
        State &B = p[i + k + 1];

        Coord jump{ B.pos.row - A.pos.row,
                    B.pos.col - A.pos.col };

        if (!legalJump(A.vel, jump)) continue;
        if (!isPathClear(A.pos, B.pos))  continue;
        if (t.at(A.pos.row, A.pos.col) == 'G' &&
            violatesGrassRule(A.vel, jump)) continue;

        std::vector<State> backup(p.begin() + i + 1, p.begin() + i + k + 1);

        p.erase(p.begin() + i + 1, p.begin() + i + k + 1);
        p[i + 1].vel = jump;

        bool ok = true;
        if (i - 1 >= 0) {
            const State& prev = p[i - 1];
            const State& from = p[i];
            Coord needVel{ from.pos.row - prev.pos.row,
                           from.pos.col - prev.pos.col };

            ok =  legalJump(prev.vel, needVel)
               && isPathClear(prev.pos, from.pos)
               && !(t.at(prev.pos.row, prev.pos.col) == 'G' &&
                    violatesGrassRule(prev.vel, needVel));
        }

        if (ok && i + 2 < (int)p.size()) {
            const State& from   = p[i + 1];
            const State& target = p[i + 2];
            Coord needVel{ target.pos.row - from.pos.row,
                           target.pos.col - from.pos.col };

            ok =  legalJump(from.vel, needVel)
               && isPathClear(from.pos, target.pos)
               && !(t.at(from.pos.row, from.pos.col) == 'G' &&
                    violatesGrassRule(from.vel, needVel));


        }

        if (!ok) {
            p.erase(p.begin() + i + 1, p.begin() + i + 2);
            p.insert(p.begin() + i + 1,
                     backup.begin(), backup.end());
        }

        break;
    }
}


void SimulatedAnnealer::tryAccelDecelStraight(std::vector<State>& p) {
    int n = (int)p.size();
    if (n < 3) return;

    struct Seg { int start, end; Coord dir; };
    std::vector<Seg> segs;

    for (int i = 0; i + 2 < n; ++i) {
        Coord d1{ p[i+1].pos.row - p[i].pos.row,
                  p[i+1].pos.col - p[i].pos.col };
        if (!d1.row && !d1.col) continue;

        int end = i + 1;
        while (end + 1 < n) {
            Coord d2{ p[end+1].pos.row - p[end].pos.row,
                      p[end+1].pos.col - p[end].pos.col };
            if (d2.row != d1.row || d2.col != d1.col) break;
            ++end;
        }
        if (end - i >= 4) {
            segs.push_back({ i, end, d1 });
            i = end - 1;  // skip ahead
        }
    }

     if (segs.empty()) return;

    std::uniform_int_distribution<int> distSeg(0, (int)segs.size() - 1);
    Seg chosen = segs[ distSeg(rng) ];
    int s = chosen.start;
    int e = chosen.end;

    int Lorig = 0;
    for (int j = s; j < e; ++j)
        Lorig += std::abs(p[j+1].pos.row - p[j].pos.row)
               + std::abs(p[j+1].pos.col - p[j].pos.col); // manhattan lengths

    if (Lorig < 1) return;

    if (Lorig % 2 == 1) {
        --e;
        --Lorig;
        if (Lorig < 1) return;
    }

    State startSt = p[s];
    State endSt   = p[e];

    Coord du {
        chosen.dir.row > 0 ?  1 : (chosen.dir.row < 0 ? -1 : 0),
        chosen.dir.col > 0 ?  1 : (chosen.dir.col < 0 ? -1 : 0)
    };

    // std::vector<int> speeds;
    // int rem = Lorig;
    // int curr = 1;
    //
    // while (true) {
    //     int sumDown = (curr - 1) * curr / 2;
    //     if (rem - curr - 1 >= sumDown) {
    //         speeds.push_back(curr);
    //         rem -= curr;
    //         curr++;
    //     }
    //     else {
    //         break;
    //     }
    // }

    int h = (int)std::floor(std::sqrt(Lorig));
    while ( (h+1)*(h+1) <= Lorig ) ++h;

    int baseDist   = h * h;
    int plateauLen = (Lorig - baseDist) / h;
    int tail       =  Lorig - baseDist - plateauLen * h;  // 0 ≤ tail < h

    // 2.  build the speed profile
    std::vector<int> speeds;
    for (int v = 1; v <= h; ++v) speeds.push_back(v);
    for (int i = 0; i < plateauLen; ++i) speeds.push_back(h);
    for (int v = h-1; v >= 1; --v) speeds.push_back(v);
    if (tail) speeds.push_back(tail);

    //while (rem > 0) {speeds.push_back(1); rem -= 1;}
    //while (rem > 0) { speeds.push_back(h); rem -= h; }


    // if (rem != 0) {
    //     return;
    // }

    int N = (int)speeds.size();
    std::vector<State> mini;
    mini.push_back(startSt);

    for (int i = 0; i < N; ++i) {
        int sp = speeds[i];
        Coord vel{ du.row * sp, du.col * sp };

        State nxt;
        nxt.pos = {
            mini.back().pos.row + vel.row,
            mini.back().pos.col + vel.col
        };
        nxt.vel = vel;

        State &prev = mini.back();
        if (t.at(prev.pos.row, prev.pos.col) == 'G' &&
            violatesGrassRule(prev.vel, vel)) {
            return;
        }
        mini.push_back(nxt);
    }


    if (mini.back().pos.col != endSt.pos.col||
        mini.back().pos.row != endSt.pos.row
        // mini.back().vel.col != du.col ||
        //  mini.back().vel.row != du.row
        ) {
        return;
    }
    p.erase(p.begin() + s + 1, p.begin() + e);
    p.insert(p.begin() + s + 1,
             mini.begin() + 1,
             mini.end()   - 1);

}

std::vector<State> SimulatedAnnealer::run(double startTemp, double coolingRate, double minTemp) {
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

    return best;
}