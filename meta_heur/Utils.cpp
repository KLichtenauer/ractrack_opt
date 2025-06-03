#include "Utils.h"
#include <cmath>   // for std::abs

namespace Utils {

    bool legalJump(const Coord &oldVel, const Coord &newVel) {
        if (std::abs(newVel.row - oldVel.row) > 1 ||
            std::abs(newVel.col - oldVel.col) > 1)
            return false;
        if (oldVel.row * newVel.row < 0 ||
            oldVel.col * newVel.col < 0)
            return false;

        return true;
    }

    bool violatesGrassRule(const Coord &oldVel, const Coord &newVel) {
        int fromX = std::abs(oldVel.row), toX = std::abs(newVel.row);
        int fromY = std::abs(oldVel.col), toY = std::abs(newVel.col);

        bool badX = (fromX >= 2 && toX >= fromX)
                 || (fromX == 1 && toX >  fromX);
        bool badY = (fromY >= 2 && toY >= fromY)
                 || (fromY == 1 && toY >  fromY);

        return badX || badY;
    }

    bool isPathClear(const Track& t, const Coord& from, const Coord& to) {
        int x1 = from.row, y1 = from.col;
        int x2 = to.row, y2 = to.col;
        int dx = std::abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
        int dy = -std::abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
        int err = dx + dy;

        while (true) {
            if (x1 < 0 || y1 < 0 || x1 >= t.height() || y1 >= t.width()) return false;
            if (t.at(x1, y1) == 'O') return false;
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

}
