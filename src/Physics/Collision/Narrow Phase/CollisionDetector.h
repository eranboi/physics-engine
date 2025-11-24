#pragma once
#include "Physics/Core/Rigidbody.h"
#include "CollisionManifold.h"

struct AxisInfo {
    sf::Vector2f normal; // normalized SAT axis
    Rigidbody* referenceBody; // this axis came from polygon A or B
    int edgeIndex; // which edge generated this axis
};

class CollisionDetector {
public:
    static bool CheckForCollision(Rigidbody* bodyA, Rigidbody* bodyB, CollisionManifold& outManifold);
    static bool PolygonVsPolygon(Rigidbody* bodyA, Rigidbody* bodyB, CollisionManifold& outManifold);
    static bool PolygonVsCircle(Rigidbody* polygon, Rigidbody* circle, CollisionManifold& outManifold);
    static bool CircleVsCircle(Rigidbody* bodyA, Rigidbody* bodyB, CollisionManifold& outManifold);

private:
    static std::vector<sf::Vector2f> GetEdgeNormals(const std::vector<sf::Vector2f>& vertices);
    static std::vector<sf::Vector2f> Clip(const std::vector<sf::Vector2f>& incFace, const sf::Vector2f& normal, float offset);
};

