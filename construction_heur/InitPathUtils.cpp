//
// Created by Privati on 23.04.25.
//

#include "InitPathUtils.h"

#include <algorithm>
#include <array>
#include <deque>
#include <iomanip>
#include <queue>
#include <random>
#include <unordered_map>
#include <unordered_set>

#include "Track.h"

static const int MAX_VELOCITY = 1;

static std::tuple<int,int,int,int> makeKey(const State &s) {
    return {s.pos.row, s.pos.col, s.vel.row, s.vel.col};
}

struct TupleHash {
    std::size_t operator()(const std::tuple<int,int,int,int> &t) const noexcept {
        auto [r, c, vr, vc] = t;
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

static std::mt19937_64 RNG(std::random_device{}());
static std::uniform_real_distribution<double> UNI(0.0, 1.0);


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

vector<vector<int>> createClearance(Track &t) {
    int H = t.height();
    int W = t.width();
    const int INF = INT_MAX;
    // Change this to 1 if you want grass to cost exactly the same as a normal cell,
    // or to 3 (etc.) for a bigger penalty.
    const int grassCost = 1;
    const int normalCost = 2;

    // distance[r][c] = minimum cost to reach any wall (“O”)
    vector<vector<int>> distance(H, vector<int>(W, INF));

    // Min‐heap of (currentDist, row, col).  We start from all walls at distance 0.
    using Elem = pair<int, pair<int,int>>;
    priority_queue<Elem, vector<Elem>, greater<Elem>> pq;

    // 1) Initialize: walls get distance 0 and go into the PQ
    for (int r = 0; r < H; ++r) {
        for (int c = 0; c < W; ++c) {
            if (t.at(r, c) == 'O') {
                distance[r][c] = 0;
                pq.push({0, {r, c}});
            }
        }
    }

    // 2) Standard Dijkstra over the 4‐connected grid
    const int dr[4] = { -1, +1,  0,  0 };
    const int dc[4] = {  0,  0, -1, +1 };

    while (!pq.empty()) {
        auto [d, rc] = pq.top();
        pq.pop();
        int r = rc.first;
        int c = rc.second;

        // If we’ve already found a better way, skip:
        if (d > distance[r][c]) continue;

        // Expand to neighbors
        for (int i = 0; i < 4; ++i) {
            int nr = r + dr[i], nc = c + dc[i];
            if (nr < 0 || nc < 0 || nr >= H || nc >= W)
                continue;

            // We never step into a wall; walls stay at distance 0, but we don't traverse them.
            if (t.at(nr, nc) == 'O')
                continue;

            // Determine the cost of entering (nr,nc):
            int stepCost = (t.at(nr, nc) == 'G' ? grassCost : normalCost);
            int newDist = d + stepCost;

            if (newDist < distance[nr][nc]) {
                distance[nr][nc] = newDist;
                pq.push({newDist, {nr, nc}});
            }
        }
    }

    return distance;
}

static vector<vector<int>> createFinishDist(const Track &t) {
    int H = t.height(), W = t.width();
    vector<vector<int>> dist(H, vector<int>(W, INT_MAX));
    deque<Coord> dq;
    for (auto &f : t.finishLine) {
        dist[f.row][f.col] = 0;
        dq.push_back(f);
    }
    static const array<pair<int,int>,4> dirs = {{
        {-1,0},{1,0},{0,-1},{0,1}
    }};
    while (!dq.empty()) {
        auto c = dq.front(); dq.pop_front();
        int d = dist[c.row][c.col];
        for (auto &Δ : dirs) {
            int nr = c.row + Δ.first, nc = c.col + Δ.second;
            if (nr < 0 || nc < 0 || nr >= H || nc >= W) continue;
            if (t.at(nr,nc) == 'O') continue;         // obstacle
            if (dist[nr][nc] <= d + 1) continue;
            dist[nr][nc] = d + 1;
            dq.emplace_back(nr, nc);
        }
    }
    return dist;
}

vector<State> InitPathUtils::initPath(Track &t) {
    const Coord startPos = t.start;
    const State startState{startPos, Coord{0, 0}};

    int H = t.height(), W = t.width();
    auto clearance = createClearance(t);
    auto finishDist = createFinishDist(t);

    double maxC = 0, maxD = 0;
    for (int r = 0; r < H; ++r) {
        for (int c = 0; c < W; ++c) {
            if (clearance[r][c] < INT_MAX) {
                maxC = max(maxC, (double)clearance[r][c]);
            }
            if (finishDist[r][c] < INT_MAX)
                maxD = max(maxD, (double)finishDist[r][c]);
        }
    }

    const double α = 0.7;
    vector<vector<double>> score(H, vector<double>(W, 0.0));
    for (int r = 0; r < H; ++r) {
        for (int c = 0; c < W; ++c) {
            double cNorm = clearance[r][c] / maxC;       // 0…1
            double dNorm = finishDist[r][c] / maxD;      // 0…1
            score[r][c] = α * cNorm + (1-α) * (1.0 - dNorm);
        }
    }

    auto cmp = [&](auto &a, auto &b) {
        return score[a.pos.row][a.pos.col]
             < score[b.pos.row][b.pos.col];
    };

    std::priority_queue<State, std::vector<State>, decltype(cmp)> q(cmp);
    std::unordered_map<std::tuple<int,int,int,int>, bool,       TupleHash, TupleEqual> visited;
    std::unordered_map<std::tuple<int,int,int,int>, State,      TupleHash, TupleEqual> parent;

    visited[makeKey(startState)] = true;
    q.push(startState);

    State goal {startPos, Coord{0, 0}};
    bool found = false;

    while (!q.empty() && !found) {
        State cur = q.top(); q.pop();
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
        int vx = cur.vel.row;
        int vy = cur.vel.col;
        int vFromX = std::abs(vx);
        int vFromY = std::abs(vy);

        for (int ax = -1; ax <= 1; ++ax) {
            int nxtVx = vx + ax;
            if ((vx > 0 && nxtVx < 0) ||
                (vx < 0 && nxtVx > 0))
                continue;

            for (int ay = -1; ay <= 1; ++ay) {
                int nxtVy = vy + ay;
                if ((vy > 0 && nxtVy < 0) ||
                    (vy < 0 && nxtVy > 0))
                    continue;

                State nxt;
                nxt.vel.row = vx + ax;
                nxt.vel.col = vy + ay;
                int speedSquared = nxt.vel.row * nxt.vel.row + nxt.vel.col * nxt.vel.col;
                if (speedSquared > MAX_VELOCITY * MAX_VELOCITY) continue;
                nxt.pos.row = cur.pos.row + nxt.vel.row;
                nxt.pos.col = cur.pos.col + nxt.vel.col;
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
    return statePath;
}


