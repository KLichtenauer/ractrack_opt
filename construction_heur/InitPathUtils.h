//
// Created by Privati on 23.04.25.
//

#ifndef INITPATH_H
#define INITPATH_H
#include <vector>

#include "State.h"
#include "Track.h"

class InitPathUtils {
public:
    static std::vector<State> initPath(Track& t);

};



#endif //INITPATH_H
