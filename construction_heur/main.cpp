//
// Created by Privati on 23.04.25.
//
#include <fstream>
#include <filesystem>
#include <iostream>
#include "Track.h"
#include "InitPathUtils.h"
using namespace std;

int main() {
    string inputFile = "track_10";
    string inputPath = "../data/programmingExercise/" + inputFile + ".t";
    Track t;
    if (!t.load(inputPath)) return 1;


    cout << "Map size: " << t.width() << " × " << t.height() << "\n";
    // e.g. print the map back out:
    for (int y = 0; y < t.height(); ++y)
        cout << t.rows[y] << "\n";


    cout << t.start.col << t.start.row;

    vector<Coord> initPath= InitPathUtils::initPath(t);

    string shortestPathOutputFilePath = "../data/init_paths/" + inputFile + ".json";

    ofstream outFile(shortestPathOutputFilePath);
    if (!outFile) {
        cerr << "Error: unable to open output file." << endl;
        return 1;
    }

    outFile << Track::to_json(initPath, t.height());

    outFile.close();

    string trackFilePath = "programmingExercise/" + inputFile + ".t";
    string tripFilePath = shortestPathOutputFilePath;
    string outputFilePath = "outputs/" + inputFile;
    string visCmd = "pwd && cd .. && cd data && pwd && ./visualise.pl " + trackFilePath + " " + tripFilePath + " " + outputFilePath;
    cout << "On console:   " + visCmd;
    system(visCmd.c_str());


    string pdfLatexCmd = "pwd && cd .. && cd data && pdflatex " +  outputFilePath;
    cout << "On Console:   " + pdfLatexCmd;
    system(pdfLatexCmd.c_str());
}
