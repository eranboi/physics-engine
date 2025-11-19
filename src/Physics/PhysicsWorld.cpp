#include "PhysicsWorld.h"

// Temporary floor positions
const float FLOOR_Y = 6.0f;
const float FLOOR_X_MAX = 8.0f;
const float FLOOR_X_MIN = 0.0f;

void PhysicsWorld::Step(float deltaTime) {
	for (Rigidbody* body : bodies) {
		body->Step(deltaTime);
		body->ApplyGravity(gravity, deltaTime);
		body->StepPosition(deltaTime);

		// Simple floor collision
		if (body->position.y + body->radius > FLOOR_Y) {
			body->position.y = FLOOR_Y - body->radius;
			body->velocity.y *= -body->restitution;
		}

		// Simple wall collision (right and left)
		if (body->position.x + body->radius > FLOOR_X_MAX) {
			body->position.x = FLOOR_X_MAX - body->radius;
			body->velocity.x *= -body->restitution;
		}
		else if (body->position.x - body->radius < FLOOR_X_MIN) {
			body->position.x = FLOOR_X_MIN + body->radius;
			body->velocity.x *= -body->restitution;
		}

		
	}
}

void PhysicsWorld::AddBody(Rigidbody* body) {
	bodies.push_back(body);
}

void PhysicsWorld::RemoveBody(Rigidbody* body) {
	auto it = std::remove(bodies.begin(), bodies.end(), body);
	bodies.erase(it, bodies.end());
}