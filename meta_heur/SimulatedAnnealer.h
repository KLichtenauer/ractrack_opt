//
// Created by vollm on 5/26/2025.
//

#ifndef SIMULATEDANNEALING_H
#define SIMULATEDANNEALING_H
#include <array>
#include <random>
#include <vector>


class Coord;
class Track;
struct State;

class SimulatedAnnealer {
public:
    SimulatedAnnealer(const Track& track, std::vector<State> initialPath);

    std::vector<State> run(double startTemp = 10.0, double coolingRate = 0.99, double minTemp = 0.1);

private:
    const Track& t;
    std::vector<State> currentPath;
    std::mt19937_64 rng;
    std::uniform_real_distribution<double> uni;
    int lastOp{-1};
    std::array<int, 3> opCounts{};

    static double computeCost(const std::vector<State>& path);
    std::vector<State> mutatePath(const std::vector<State>& path);

    void tryTwoStepShortcut(std::vector<State> &p);
    void tryThreeStepShortcut(std::vector<State> &p);
    void tryInsertStep(std::vector<State> &p);
    void tryAccelDecelStraight(std::vector<State> &p);

    bool isInside(const Coord& c) const;
    bool isPathClear(const Coord& from, const Coord& to) const;
};


#endif //SIMULATEDANNEALING_H
