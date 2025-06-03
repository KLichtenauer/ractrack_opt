//
// Created by Privati on 03.06.25.
//

#ifndef GENETICOPTIMIZER_H
#define GENETICOPTIMIZER_H



#pragma once
#include <vector>
#include <random>
#include "../construction_heur/Track.h"
#include "SimulatedAnnealer.h"   // for State, Coord and helpers

class GeneticOptimizer {
public:
    GeneticOptimizer(const Track& track,
                     std::vector<State> seedPath,
                     int popSize   = 60,
                     int eliteKeep = 10);

    std::vector<State> run(int maxGenerations = 400);

private:

    // --- helpers ---
    double fitness(const std::vector<State>& p) const;
    std::vector<State> crossover(const std::vector<State>& a,
                                 const std::vector<State>& b);
    void mutate(std::vector<State>& individual);

    // --- data ---
    const Track& t;
    int POP, ELITE;
    std::mt19937_64 rng;
    std::uniform_real_distribution<double> uni;
    std::vector<std::vector<State>> population;   // current pop
};

#endif //GENETICOPTIMIZER_H
