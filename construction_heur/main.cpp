//
// Created by Privati on 23.04.25.
//
#include <fstream>
#include <filesystem>
#include <iostream>
#include <vector>

#include "Track.h"
#include "InitPathUtils.h"
using namespace std;

int main() {
    vector<string> inputFiles = {"track_02", "track_03", "track_04", "track_05", "track_06", "track_07", "track_08", "track_09", "track_10"};

    for (int i = 0; i < inputFiles.size(); i++) {
        string inputFile = inputFiles[i];
        string inputPath = "../data/programmingExercise/" + inputFile + ".t";

        auto t0 = chrono::high_resolution_clock::now();
        Track t;
        if (!t.load(inputPath)) return 1;

        vector<Coord> initPath= InitPathUtils::initPath(t);
        auto t1 = chrono::high_resolution_clock::now();
        double durationSeconds = chrono::duration<double>(t1 - t0).count();

        string shortestPathOutputFilePath = "../data/init_paths/" + inputFile + ".json";
        ofstream outFile(shortestPathOutputFilePath);
        if (!outFile) {
            cerr << "Error: unable to open output file." << endl;
            return 1;
        }

        outFile << Track::to_json(initPath, t.height());

        cout << "Track: " << inputFile << ";\t duration: " << durationSeconds << ";\t steps:" << initPath.size() << endl;

        outFile.close();

    }


}
