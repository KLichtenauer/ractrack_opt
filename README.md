# Optimization in Transport and Logistics Submission

## Construction Heuristic
The construction heuristic is located in `./construction_heur/InitPathUtils.cpp`.
Our construction heuristic first creates a map based on the distance to the finish line and a map denoting the distance
to walls where grass steps are more expensive. It then creates a priority queue with a comparator that weighs both of 
these maps by a value `α` where `α = 0` only considers the distance to the finish line and `α = 1` only considers the
distance to walls. We have found that for most tracks a value of `α = 0.7` works best, though some tracks work better 
with either only considering distance to walls or only distance to the finish line. A BFS algorithm is then runs on this
priority queue.

## Metaheuristic Optimization
The metaheuristic optimization is located in `./meta_heur/SimulatedAnnealer.cpp`. We have decided to use a Simulated
Annealer for our optimization process. Here we mutate the path in each iteration with three different mutations:
- `tryTwoStepShortcut()`: This mutation tries to skip one step by checking whether `step[i] -> step[i+2]` is a legal move.
- `tryThreeStepShortcut()`: This mutation tries to skip two steps by checking whether `step[i] -> step[i+3]` is a legal move.
- `tryAccelDecelStraight()`: This mutation first creates a list of all straight line segments and then tries to accelerate as much as it can before decelerating again.

## Running the Program
### Prerequisites

- **Compiler & Build Tools**
    - A C++ compiler with **C++20** support (e.g. GCC 10+, Clang 12+, MSVC 2019+).
    - [CMake ≥ 3.30](https://cmake.org/).
    - (Optional) Ninja or Make for faster builds.

### Building (Release Mode)

```bash
# 2. Create & enter a release build directory
mkdir cmake-build-release && cd cmake-build-release

# 3. Generate build files in Release mode
cmake -DCMAKE_BUILD_TYPE=Release ..

# 4. Compile
cmake --build .
```

The resulting executable will be `cmake-build-release/racetrack_opt`.

### Running

From the `cmake-build-release/` directory, simply run:

```bash
./racetrack_opt
```

This will:

1. Load each track `track_02.t` … `track_10.t` from `../data/programmingExercise/`
2. Compute an initial path and optimize it via simulated annealing.
3. Write each optimized path as JSON to `../data/init_paths/<track>.json`
4. Invoke `visualise.pl` and `pdflatex` to generate a PDF in `../data/outputs/<track>.pdf`
5. Print a summary table (track name, runtime, number of steps) to stdout.

### Output

- **JSON paths** → `data/init_paths/track_##.json`
- **Visual PDFs** → `data/outputs/track_##.pdf`
- **Console summary** after processing all tracks.
