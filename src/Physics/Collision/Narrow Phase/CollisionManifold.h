#pragma once
#include "Physics/Core/Rigidbody.h"

struct CollisionManifold {
    // Bodies involved
    Rigidbody* refBody;
    Rigidbody* incBody;

    // Contact info
    std::vector<sf::Vector2f> contactPoints;
    sf::Vector2f collisionAxis;
    float depth;

};


