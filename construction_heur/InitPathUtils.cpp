//
// Created by Privati on 23.04.25.
//

#include "InitPathUtils.h"

#include <algorithm>
#include <deque>
#include <iomanip>
#include <queue>
#include <unordered_map>

#include "Track.h"

struct State {
    Coord pos;
    Coord vel;
    State() : pos(0,0), vel(0,0) {}
    State(const Coord &p, const Coord &v) : pos(p), vel(v) {}
};

static std::tuple<int,int,int,int> makeKey(const State &s) {
    return {s.pos.row, s.pos.col, s.vel.row, s.vel.col};
}

struct TupleHash {
    std::size_t operator()(const std::tuple<int,int,int,int> &t) const noexcept {
        auto [r, c, vr, vc] = t;
        // combine the four ints into one hash
        std::size_t h = std::hash<int>{}(r);
        h = h * 31 + std::hash<int>{}(c);
        h = h * 31 + std::hash<int>{}(vr);
        h = h * 31 + std::hash<int>{}(vc);
        return h;
    }
};

struct TupleEqual {
    bool operator()(const std::tuple<int,int,int,int> &a,
                    const std::tuple<int,int,int,int> &b) const noexcept {
        return a == b;
    }
};

static std::vector<State> reconstructPath(
    const State &endState,
    const std::unordered_map<std::tuple<int, int, int, int>, State, TupleHash, TupleEqual> &parentMap) {
    std::vector<State> path;
    State curr = endState;
    path.push_back(curr);
    auto key = makeKey(curr);
    while (parentMap.count(key)) {
        curr = parentMap.at(key);
        path.push_back(curr);
        key = makeKey(curr);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

static bool isPathClear(const Track &t, const Coord &from, const Coord &to) {
    int x1 = from.row,  y1 = from.col;
    int x2 = to.row,    y2 = to.col;
    int dx =  std::abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
    int dy = -std::abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (x1 < 0 || x1 >= t.height() ||
            y1 < 0 || y1 >= t.width()  ||
            t.at(x1, y1) == 'O')
            return false;

        if (x1 == x2 && y1 == y2)
            break;

        int oldX = x1, oldY = y1;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x1 += sx; }
        if (e2 <= dx) { err += dx; y1 += sy; }

        if (x1 != oldX && y1 != oldY) {
            if (t.at(oldX, y1) == 'O' ||
                t.at(x1, oldY) == 'O')
                return false;
        }
    }

    return true;
}


vector<vector<int> > createClearance(Track &t) {
    deque<Coord> qClearance;
    vector clearance(t.height(), vector<int>(t.width(), INT_MAX));

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
    return clearance;
}

vector<Coord> InitPathUtils::initPath(Track &t) {
    const Coord startPos = t.start;
    const State startState{startPos, Coord{0, 0}};
    queue<State> q;
    std::unordered_map<std::tuple<int,int,int,int>, bool,       TupleHash, TupleEqual> visited;
    std::unordered_map<std::tuple<int,int,int,int>, State,      TupleHash, TupleEqual> parent;

    visited[makeKey(startState)] = true;
    q.push(startState);

    State goal {startPos, Coord{0, 0}};
    bool found = false;

    while (!q.empty() && !found) {
        State cur = q.front(); q.pop();
        for (auto &f : t.finishLine) {
            if (cur.pos.row == f.row && cur.pos.col == f.col) {
                goal = cur;
                found = true;
                break;
            }
        }
        if (found) {
            break;
        }

        bool onGrass = (t.at(cur.pos.row, cur.pos.col) == 'G');
        int vFromX = std::abs(cur.vel.row);
        int vFromY = std::abs(cur.vel.col);

        for (int ax = -1; ax <= 1; ++ax) {
            for (int ay = -1; ay <= 1; ++ay) {
                int nextVelX = cur.vel.row + ax;
                int nextVelY = cur.vel.col + ay;
                int nextPosX = cur.pos.row + nextVelX;
                int nextPosY = cur.pos.col + nextVelY;
                State nxt{{nextPosX, nextPosY}, {nextVelX, nextVelY}};
                if (nxt.pos.row < 0 || nxt.pos.col < 0 || nxt.pos.row >= t.height() || nxt.pos.col >= t.width()) continue;
                if (!isPathClear(t, cur.pos, nxt.pos)) continue;
                if (onGrass) {
                    int vToX = std::abs(nxt.vel.row);
                    int vToY = std::abs(nxt.vel.col);
                    bool badX = (vFromX >= 2 && vToX >= vFromX)
                             || (vFromX == 1 && vToX >  vFromX);
                    bool badY = (vFromY >= 2 && vToY >= vFromY)
                             || (vFromY == 1 && vToY >  vFromY);
                    if (badX || badY)
                        continue;
                }
                auto key = makeKey(nxt);
                if (visited[key]) continue;
                visited[key] = true;
                parent[key] = cur;
                q.push(nxt);
            }
        }
    }

    if (!found) {
        return {};
    }
    auto statePath = reconstructPath(goal, parent);
    vector<Coord> path;
    for (auto &s : statePath) path.push_back(s.pos);
    return path;
}
