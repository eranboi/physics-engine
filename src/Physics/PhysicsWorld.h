#pragma once
#include <SFML/System/Vector2.hpp>
#include <vector>
#include "Rigidbody.h"


class PhysicsWorld {
public:
	PhysicsWorld() {
		gravity = sf::Vector2f(0.0f, 9.81f);
	}

	// Update physics world (e.g., apply gravity to all rigidbodies)
	void Step(float deltaTime);
	void AddBody(Rigidbody* body);
	void RemoveBody(Rigidbody* body);


private:
	std::vector<Rigidbody*> bodies;
	sf::Vector2f gravity;
	void ResolveCollisions();
	void BoundaryCheck(float deltaTime);


};