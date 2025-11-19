#include "PhysicsWorld.h"
#include <cmath>

// Temporary floor positions
const float FLOOR_Y = 9.0f;
const float FLOOR_X_MAX = 16.0f;
const float FLOOR_X_MIN = 0.0f;
const int solverIterations = 8;

void PhysicsWorld::Step(float deltaTime) {

	for (Rigidbody* body : bodies) {
		body->Step(deltaTime);
		body->ApplyGravity(gravity, deltaTime);
		body->StepPosition(deltaTime);
	}

	for (int i = 0; i < solverIterations; i++)
	{
		ResolveCollisions();
		BoundaryCheck(deltaTime);
	}

	
}

void PhysicsWorld::ResolveCollisions() {
	for (int i = 0; i < bodies.size(); i++)
	{
		for (int j = i + 1; j < bodies.size(); j++)
		{
			Rigidbody* bodyA = bodies[i];
			Rigidbody* bodyB = bodies[j];

			sf::Vector2f collisionNormal = bodyB->position - bodyA->position;

			// Find the distance between bodies using Euclidean Distance 
			// Optimization: use distance squared to avoid sqrt until necessary
			float distanceSq = collisionNormal.x * collisionNormal.x + collisionNormal.y * collisionNormal.y;
			float totalRadius = bodyA->radius + bodyB->radius;
			float totalRadiusSq = totalRadius * totalRadius;

			// If the distance is smaller than the total radius, they're colliding.
			// If they're colliding, find the MTV and move the bodies accordingly.
			if (distanceSq < totalRadiusSq) {

				// Calculate actual distance
				float distance = std::sqrt(distanceSq);

				float penetrationDepth = totalRadius - distance;
				sf::Vector2f collisionNormalNormalized = collisionNormal / distance;

				// Position correction 
				bodyA->position -= collisionNormalNormalized * (penetrationDepth * 0.5f);
				bodyB->position += collisionNormalNormalized * (penetrationDepth * 0.5f);

				// Find the relvative velocity
				sf::Vector2f relativeVelocity = bodyB->velocity - bodyA->velocity;

				// Find the velocity along the normal of the collision, using dot product.
				float velocityAlongNormal = relativeVelocity.x * collisionNormalNormalized.x + relativeVelocity.y * collisionNormalNormalized.y;

				// Get the minimum restitution of the two bodies
				float e = std::min(bodyA->restitution, bodyB->restitution);

				// Calculate impulse scalar
				float j = (-(1.0f + e) * velocityAlongNormal) / (bodyA->invMass + bodyB->invMass);

				// Calculate the impulse
				sf::Vector2f impulse = j * collisionNormalNormalized;


				bodyA->velocity -= impulse * bodyA->invMass;
				bodyB->velocity += impulse * bodyB->invMass;
			}
		}
	}
}

void PhysicsWorld::BoundaryCheck(float deltaTime) {

	for (Rigidbody* body : bodies) {
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