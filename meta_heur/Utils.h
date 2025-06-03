

#ifndef UTILS_H
#define UTILS_H

#include "../construction_heur/Track.h"
#include "../construction_heur/InitPathUtils.h"

namespace Utils {

    bool legalJump(const Coord &oldVel, const Coord &newVel);

    bool isPathClear(const Track& t, const Coord& from, const Coord& to);

    bool violatesGrassRule(const Coord &oldVel, const Coord &newVel);

}



#endif //UTILS_H
