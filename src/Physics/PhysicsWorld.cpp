#include "PhysicsWorld.h"

void PhysicsWorld::Step(float deltaTime) {
	for (Rigidbody* body : bodies) {
		body->ApplyGravity(gravity, deltaTime);
		body->StepPosition(deltaTime);
	}
}

void PhysicsWorld::AddBody(Rigidbody* body) {
	bodies.push_back(body);
}

void PhysicsWorld::RemoveBody(Rigidbody* body) {
	auto it = std::remove(bodies.begin(), bodies.end(), body);
	bodies.erase(it, bodies.end());
}