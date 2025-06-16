//
// Created by Privati on 23.04.25.
//
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>

#include "Track.h"
#include "InitPathUtils.h"
#include "../meta_heur/SimulatedAnnealer.h"
using namespace std;

int main() {
    vector<string> inputFiles = {"track_02", "track_03", "track_04", "track_05", "track_06", "track_07", "track_08", "track_09", "track_10"};
    struct Summary {
        string name;
        double time;
        size_t steps;
    };
    vector<Summary> summaries;

    // Loop over input files
    for (int i = 0; i < inputFiles.size(); i++) {
        string inputFile = inputFiles[i];
        string inputPath = "../data/programmingExercise/" + inputFile + ".t";

        // Start timer
        auto t0 = chrono::high_resolution_clock::now();
        Track t;
        if (!t.load(inputPath)) return 1;

        // Run the construction heuristic
        vector<State> initPath = InitPathUtils::initPath(t);

        // Run the metaheuristic optimization
        SimulatedAnnealer simulated_annealer(t, initPath);
        vector<State> result = simulated_annealer.run(10000, 0.995, 0.01);

        // Stop the timer
        auto t1 = chrono::high_resolution_clock::now();
        double durationSeconds = chrono::duration<double>(t1 - t0).count();

        string shortestPathOutputFilePath = "../data/init_paths/" + inputFile + ".json";
        ofstream outFile(shortestPathOutputFilePath);
        if (!outFile) {
            cerr << "Error: unable to open output file." << endl;
            return 1;
        }

        // Save track summary
        summaries.push_back({ inputFile, durationSeconds, result.size() });

        // Write results to pdf
        outFile << Track::to_json(result, t.height());
        outFile.close();
        string trackFilePath = "programmingExercise/" + inputFile + ".t";
        string tripFilePath = shortestPathOutputFilePath;
        string outputFilePath = "outputs/" + inputFile;
        string visCmd   = "perl ../data/visualise.pl ../data/programmingExercise/"
                        + inputFile + ".t "
                        + tripFilePath + " ../data/outputs/" + inputFile;
        system(visCmd.c_str());
        string pdfLatexCmd = "cd .. && cd data && pdflatex "+  outputFilePath;
        system(pdfLatexCmd.c_str());


    }

    cout << "\n=== Track Summaries ===\n";
    cout << left
         << setw(12) << "Track"
         << setw(12) << "Time (s)"
         << setw(12) << "Steps"
         << "\n";
    for (auto &s : summaries) {
        cout << setw(12) << s.name
             << setw(12) << s.time
             << setw(12) << s.steps - 1
             << "\n";
    }


}
