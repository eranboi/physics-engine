#pragma once
#include <vector>
#include "Physics/Collision/Narrow Phase/CollisionManifold.h"

class ISolver {
public:
    virtual ~ISolver() = default;
    virtual void Solve(std::vector<CollisionManifold>& manifolds, float dt) = 0;
};
