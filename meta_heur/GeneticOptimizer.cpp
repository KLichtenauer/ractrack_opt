#include "GeneticOptimizer.h"

#include <algorithm>
#include <__random/random_device.h>

#include "SimulatedAnnealer.h"
#include "Utils.h"

class Track;
struct State;
// ---------- ctor ------------------------------------------------------------
GeneticOptimizer::GeneticOptimizer(const Track& track,
                                   std::vector<State> seed,
                                   int popSize, int eliteKeep)
    : t(track),
      POP(popSize),
      ELITE(eliteKeep),
      rng(std::random_device{}()),
      uni(0.0,1.0)
{
    // create initial population
    population.reserve(POP);
    population.push_back(std::move(seed));

    SimulatedAnnealer tmpSA(t, population.front());
    for (int i = 1; i < POP; ++i) {
        auto ind = tmpSA.mutatePath(population.front());   // legal randomisation
        population.push_back(std::move(ind));
    }
}

// ---------- small helpers ---------------------------------------------------
double GeneticOptimizer::fitness(const std::vector<State>& p) const {
    return 1.0 / (1.0 + p.size());             // shorter ⇒ larger
}


// single-point splice, then trim illegal tail if needed
std::vector<State>
GeneticOptimizer::crossover(const std::vector<State>& a,
                             const std::vector<State>& b)
{
    if (a.size() < 3 || b.size() < 3)    // degenerate cases
        return a.size()<b.size()?a:b;

    std::uniform_int_distribution<int> cutA(1,(int)a.size()-2);
    std::uniform_int_distribution<int> cutB(1,(int)b.size()-2);
    int i = cutA(rng);
    int j = cutB(rng);

    std::vector<State> child;
    child.reserve(a.size()+b.size());
    child.insert(child.end(), a.begin(), a.begin()+i);
    child.insert(child.end(), b.begin()+j, b.end());

    // legality repair: drop tail while wrong
    while (child.size()>=3) {
        auto& X = child[child.size()-3];
        auto& Y = child[child.size()-2];
        auto& Z = child.back();

        if (Utils::legalJump(X.vel,{Y.pos.row-X.pos.row,Y.pos.col-X.pos.col}) &&
            Utils::legalJump(Y.vel,{Z.pos.row-Y.pos.row,Z.pos.col-Y.pos.col}) &&
            Utils::isPathClear(t, X.pos, Y.pos) &&
            Utils::isPathClear(t, Y.pos, Z.pos))
            break;

        child.pop_back();         // drop last, keep trying
    }
    return child;
}

void GeneticOptimizer::mutate(std::vector<State>& ind) {
    SimulatedAnnealer tmp(t, ind);
    ind = tmp.mutatePath(ind);    // uses your existing 4 operators
}

// ---------- main loop -------------------------------------------------------
std::vector<State> GeneticOptimizer::run(int maxGen)
{
    int stagnate = 0;
    double bestFit = fitness(population[0]);

    for (int gen = 0; gen < maxGen && stagnate < 50; ++gen)
    {
        // ---- produce children --------------------------------------------
        std::vector<std::vector<State>> offspring;
        offspring.reserve(POP-ELITE);

        auto tournament = [&](void)->int{
            std::uniform_int_distribution<int> pick(0,POP-1);
            int best = pick(rng);
            for(int k=1;k<3;++k){
                int c = pick(rng);
                if (fitness(population[c]) > fitness(population[best]))
                    best = c;
            }
            return best;
        };

        while ((int)offspring.size() < POP-ELITE) {
            int pa = tournament();
            int pb = tournament();
            auto child = crossover(population[pa], population[pb]);
            mutate(child);
            offspring.push_back(std::move(child));
        }

        // ---- elitist replacement ----------------------------------------
        std::sort(population.begin(), population.end(),
                  [&](auto& x, auto& y){return fitness(x)>fitness(y);} );

        population.resize(ELITE);                      // keep elites only
        population.insert(population.end(),
                          offspring.begin(), offspring.end());

        // ---- update stagnation counter ----------------------------------
        double genBest = fitness(population[0]);
        if (genBest > bestFit+1e-9) { bestFit=genBest; stagnate=0; }
        else                         { ++stagnate; }
    }

    std::sort(population.begin(), population.end(),
              [&](auto& x, auto& y){return fitness(x)>fitness(y);} );
    return population.front();                 // shortest path found
}
