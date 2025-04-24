//
// Created by Privati on 23.04.25.
//

#ifndef TRACK_H
#define TRACK_H

#include <string>
#include <fstream>
#include <iostream>

using namespace std;


class Coord {
public:
  int row;
  int col;

  Coord(int r, int c)
    : row(r),
      col(c) {
  }
};


class Step {
public:
  Coord current;
  Coord before;
  int velocity;

  Step(Coord current_val, Coord before_val, int velocity_val): current(current_val), before(before_val),
                                                               velocity(velocity_val) {
  }
};

class Track {
public:
  vector<string> rows;
  Coord start;
  vector<Coord> finishLine;

  int width() const { return rows.empty() ? 0 : rows[0].size(); }
  int height() const { return rows.size(); }

  bool foundF(Coord &c) {
    return rows[c.row][c.col] == 'F';
  }

  Track(): start(Coord(0, 0)) {
  }


  bool load(const string &filename) {
    ifstream in(filename);
    if (!in) {
      std::cerr << "Error opening " << filename << "\n";
      return false;
    }
    string line;
    while (getline(in, line)) {
      if (!line.empty() && line.back() == '\r')
        line.pop_back();
      rows.push_back(line);
    }

    // Find start and set it for variable.

    for (int row = 0; row < height(); row++) {
      for (int column = 0; column < width(); column++) {
        if (rows[row][column] == 'S') {
          start = Coord(row, column);
        }
        if (rows[row][column] == 'F') {
          finishLine.emplace_back(row, column);
          cout << "FINISH LINE FOUND: " << row << column << "\n";
        }
      }
    }
    return true;
  }

  char at(int row, int column) const {
    return rows[row][column];
  }

  //static string to_json(const vector<Coord> &path, int height);
  static string to_json(const vector<Coord>& path, const int height);

};


#endif //TRACK_H
