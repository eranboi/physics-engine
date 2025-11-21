#include "PhysicsWorld.h"
#include <cmath>
#include "./Collision/BroadPhase/Grid.h"
#include <SFML/System/Vector2.hpp>
#include "./Utils/MathUtils.h"
#include <iostream>

// Temporary floor positions based on window size
const float FLOOR_Y = 9.0f;
const float FLOOR_X_MAX = 16.0f;
const float FLOOR_X_MIN = 0.0f;
const int solverIterations = 16;

void PhysicsWorld::Step(float deltaTime) {
	// Reset the grid
	grid.Clear();
	for (Rigidbody* body : bodies) {
		body->Step(deltaTime);
		body->ApplyGravity(gravity, deltaTime);
		body->StepPosition(deltaTime);

		// Fill the grid with bodies
		grid.AddBody(body);
	}

	for (int i = 0; i < solverIterations; i++)
	{
		ResolveCollisions();
	}
}

void PhysicsWorld::ResolveCollisions() {
	for(Rigidbody* bodyA : bodies)
	{
		// Another collision check for the walls
		for (Rigidbody* wall : staticBodies) {
			CheckForCollision(*bodyA, *wall);
		}

		// Get the cell of the body A
		int cellX = static_cast<int>(bodyA->position.x / grid.GetCellSize());
		int cellY = static_cast<int>(bodyA->position.y / grid.GetCellSize());

		// Iterate on the neighbor cells
		for (int x = -1; x <= 1; x++) {
			for (int y = -1; y <= 1; y++) {

				// Get the bodies of the neighbor cell
				const auto& neighbors = grid.GetCellContent(cellX + x, cellY + y);

				for (Rigidbody* bodyB : neighbors) {

					// Skip self
					if (bodyA == bodyB) continue;

					// Duplicate check, if A-B is resolved skip B-A
					if (bodyA > bodyB) continue;

					
					if (bodyA->shapeType == ShapeType::Circle && bodyB->shapeType == ShapeType::Circle) {
						CheckForCircleCircleCollision(bodyA, bodyB);
					}
					else {
						CheckForCollision(*bodyA, *bodyB);
					}
				}
			}
		}
	}
}

void PhysicsWorld::AddBody(Rigidbody* body) {
	if (body->invMass == 0.0f) {
		staticBodies.push_back(body);
	}
	else {
		bodies.push_back(body);
	}
}

void PhysicsWorld::RemoveBody(Rigidbody* body) {
	if (body->invMass == 0.0f) {
		auto it = std::remove(staticBodies.begin(), staticBodies.end(), body);
		staticBodies.erase(it, staticBodies.end());
	}
	else {
		auto it = std::remove(bodies.begin(), bodies.end(), body);
		bodies.erase(it, bodies.end());
	}
}

// SAT collision detection
void PhysicsWorld::CheckForCollision(Rigidbody& bodyA, Rigidbody& bodyB) {

	// Get the vertices of the bodies
	std::vector<sf::Vector2f>& verticesA = bodyA.GetTransformedVertices();
	std::vector<sf::Vector2f>& verticesB = bodyB.GetTransformedVertices();

	// Get the axes to test the shapes against. 
	std::vector<sf::Vector2f> axesA = GetAxes(verticesA);
	std::vector<sf::Vector2f> axesB = GetAxes(verticesB);

	// Combine the axes in a vector
	std::vector<sf::Vector2f> axes;
	for (auto& a : axesA) {
		axes.push_back(a);
	}

	for (auto& a : axesB) {
		axes.push_back(a);
	}

	float mtv = 999.0f;
	sf::Vector2f mtvAxis;
	bool colliding = false;

	// Project all the vertices onto the axes. 
	// Get the min and max values for both shapes.
	// If we find an axis where they do NOT overlap, they're not colliding.
	for (auto axis : axes) {
		float minA = std::numeric_limits<float>::max();
		float maxA = std::numeric_limits<float>::lowest();

		float minB = std::numeric_limits<float>::max();
		float maxB = std::numeric_limits<float>::lowest();

		for (int i = 0; i < verticesA.size(); i++)
		{
			sf::Vector2f vert = verticesA[i];

			float dotProduct = MathUtils::Dot(vert, axis);
			if (dotProduct <= minA) {
				minA = dotProduct;
			}
			if (dotProduct >= maxA)
			{
				maxA = dotProduct;
			}
		}

		for (int i = 0; i < verticesB.size(); i++)
		{
			sf::Vector2f vert = verticesB[i];

			float dotProduct = MathUtils::Dot(vert, axis);
			if (dotProduct <= minB) {
				minB = dotProduct;
			}
			if (dotProduct >= maxB)
			{
				maxB = dotProduct;
			}
		}

		// if the min value of something is bigger than the other
		// that means that is the seperating axis. No collision.
		if (minA > maxB || minB > maxA) {
			colliding = false;
			break;
		}

		colliding = true;

		// To find the overlap, we should get the minimum max value and maximum min value
		// then we subtract the maxMin from minMax 
		float minMax = std::min(maxA, maxB);
		float maxMin = std::max(minA, minB);
		
		float tempMtv = minMax - maxMin;

		if (tempMtv < mtv) 
		{
			mtv = tempMtv;
			mtvAxis = axis;
		}
	}

	sf::Vector2f collisionNormal = bodyB.position - bodyA.position;

	float mtvDirDot = MathUtils::Dot(mtvAxis, collisionNormal);
	if (mtvDirDot < 0) mtvAxis = sf::Vector2f(-mtvAxis.x, -mtvAxis.y);

	if (colliding) {
		ResolveCollision(bodyA, bodyB, mtv, mtvAxis);
	}
}

// Basic circle-circle collision detection
void PhysicsWorld::CheckForCircleCircleCollision(Rigidbody* bodyA, Rigidbody* bodyB) {
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

		// sf::Vector2f collisionNormalNormalized = collisionNormal / distance;

		ResolveCollision(*bodyA, *bodyB, penetrationDepth, collisionNormal);
	}
}

void PhysicsWorld::ResolveCollision(Rigidbody& bodyA, Rigidbody& bodyB, float mtv, sf::Vector2f collisionNormal) {

	sf::Vector2f collisionNormalNormalized = MathUtils::Normalize(collisionNormal);

	// Positional correction based on mass
	float totalInvMass = bodyA.invMass + bodyB.invMass;
	if (totalInvMass > 0.0f)
	{
		sf::Vector2f movePerIMass = collisionNormalNormalized * (mtv / totalInvMass);
		bodyA.position -= movePerIMass * bodyA.invMass;
		bodyB.position += movePerIMass * bodyB.invMass;
	}

	// Find the relvative velocity
	sf::Vector2f relativeVelocity = bodyB.velocity - bodyA.velocity;

	// Find the velocity along the normal of the collision, using dot product.
	float velocityAlongNormal = MathUtils::Dot(relativeVelocity, collisionNormalNormalized);

	// Get the minimum restitution of the two bodies
	float e = std::min(bodyA.restitution, bodyB.restitution);

	// Calculate impulse scalar
	float j = (-(1.0f + e) * velocityAlongNormal) / (bodyA.invMass + bodyB.invMass);

	// Calculate the impulse
	sf::Vector2f impulse = j * collisionNormalNormalized;


	bodyA.velocity -= impulse * bodyA.invMass;
	bodyB.velocity += impulse * bodyB.invMass;
}

std::vector<sf::Vector2f> PhysicsWorld::GetAxes(const std::vector<sf::Vector2f> &vertices) {
	std::vector<sf::Vector2f> axes;

	// Find the axis by getting the vertices.
	for (int i = 0; i < vertices.size(); i++) {
		sf::Vector2f p1 = vertices[i];
		sf::Vector2f p2 = vertices[(i + 1) % vertices.size()];

		// calculate the vector representing the edge using the two vertices
		sf::Vector2f edge = p2 - p1;

		// calculate the perpendicular vector (90 degrees anti-clockwise) 
		// if we want to do it clockwise we can simply do (edge.y, -edge.x)
		// in our case it does not matter.
		sf::Vector2f edgeNormal = sf::Vector2f(-edge.y, edge.x);

		// normalize the perpendecular
		sf::Vector2f normalNormalized = MathUtils::Normalize(edgeNormal);

		axes.push_back(normalNormalized);
	}

	return axes;
}