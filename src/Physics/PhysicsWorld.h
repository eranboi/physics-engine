#pragma once
#include <SFML/System/Vector2.hpp>
#include <vector>
#include "Rigidbody.h"
#include "./Collision/BroadPhase/Grid.h"
#include "./Debugging/Gizmos.h"


struct AxisInfo {
	sf::Vector2f normal; // normalized SAT axis
	Rigidbody* referenceBody; // this axis came from polygon A or B
	int edgeIndex; // which edge generated this axis
};

struct CollisionManifold {
	// Bodies involved
	Rigidbody* refBody;
	Rigidbody* incBody;

	// Contact info
	std::vector<sf::Vector2f> contactPoints;
	sf::Vector2f collisionAxis;
	float depth;

};

class PhysicsWorld {
public:
	PhysicsWorld(const int worldWidth, const int worldHeight) : grid(worldWidth, worldHeight, 0.25f) 
	{
		gravity = sf::Vector2f(0.0f, 9.81f);
	}

	// Update physics world (e.g., apply gravity to all rigidbodies)
	void Step(float deltaTime);
	void AddBody(Rigidbody* body);
	void RemoveBody(Rigidbody* body);
private:
	std::vector<Rigidbody*> bodies;
	std::vector<Rigidbody*> staticBodies;
	sf::Vector2f gravity;
	void ResolveCollisions() const;

	static void CheckForCollision(Rigidbody& bodyA, Rigidbody& bodyB);

	static void CheckForCircleCircleCollision(const Rigidbody* bodyA, const Rigidbody* bodyB);

	static void ResolveCollision(const CollisionManifold &manifold);

	static std::vector<sf::Vector2f> GetAxes(const std::vector<sf::Vector2f>& vertices);

	static std::vector<sf::Vector2f> Clip(const std::vector<sf::Vector2f>& incFace, const sf::Vector2f& normal, float offset);

	Grid grid;

};