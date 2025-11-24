#pragma once
#include <SFML/System/Vector2.hpp>
#include <vector>
#include "Core/Rigidbody.h"
#include "./Collision/BroadPhase/Grid.h"
#include "Dynamics/Solvers/ISolver.h"

class PhysicsWorld {
public:
	PhysicsWorld(const int worldWidth, const int worldHeight, ISolver* initialSolver)
		: grid(worldWidth, worldHeight, 0.3f), solver(initialSolver)
	{
		gravity = sf::Vector2f(0.0f, 9.81f);
	}

	~PhysicsWorld() {
		delete solver;
	}

	// Update physics world
	void Step(float deltaTime);
	void AddBody(Rigidbody* body);
	void RemoveBody(Rigidbody* body);

	int GetManifoldCount() const { return lastManifoldCount; }
private:
	int lastManifoldCount = 0;
	std::vector<Rigidbody*> bodies;
	std::vector<Rigidbody*> staticBodies;
	sf::Vector2f gravity;

	Grid grid;
	ISolver* solver;
};
