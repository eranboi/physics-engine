#pragma once
#include "ISolver.h"
#include <vector>

class ImpulseSolver final : public ISolver {
public:
    static void ResolveVelocity(const CollisionManifold & manifold);
    static void ApplyPositionalCorrection(const CollisionManifold & manifold);
    void Solve(std::vector<CollisionManifold>& manifolds, float dt) override;
};