#include "PhysicsWorld.h"
#include <cmath>
#include "./Collision/BroadPhase/Grid.h"
#include <SFML/System/Vector2.hpp>
#include <algorithm>
#include "Collision/Narrow Phase/CollisionDetector.h"

void PhysicsWorld::Step(const float deltaTime) {
	// Integration and Broad Phase Update
	grid.Clear();
	for (Rigidbody* body : bodies) {
		body->Step(deltaTime);
		body->ApplyGravity(gravity, deltaTime);

		// Fill the grid with bodies
		grid.AddBody(body);
	}

    // Narrow Phase Collision Detection
    // Collision data will be stored here
    std::vector<CollisionManifold> manifolds;

	// Check for collisions between dynamic bodies and static walls
	for (Rigidbody* bodyA : bodies) {
        // Check for walls (static bodies)
		for (Rigidbody* wall : staticBodies) {
            CollisionManifold manifold;
			if (CollisionDetector::CheckForCollision(bodyA, wall, manifold)) {
                manifolds.push_back(manifold);
			}
		}

		// Get the cell of the body A
		const int cellX = static_cast<int>(bodyA->position.x / grid.GetCellSize());
		const int cellY = static_cast<int>(bodyA->position.y / grid.GetCellSize());

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

                    CollisionManifold manifold;
                    // Use the static CollisionDetector
                    // You must implement a Circle-Polygon check inside CollisionDetector if needed
					if (CollisionDetector::CheckForCollision(bodyA, bodyB, manifold)) {
                        manifolds.push_back(manifold);
					}
				}
			}
		}
	}
    // Collision Resolution
    if (solver) {
        solver->Solve(manifolds, deltaTime);
    }
}

void PhysicsWorld::AddBody(Rigidbody* body) {
	// If the body has infinite mass (invMass == 0), treat it as static.
	if (body->invMass == 0.0f) {
		staticBodies.push_back(body);
	}
	// Otherwise, treat it as dynamic.
	else {
		bodies.push_back(body);
	}
}

void PhysicsWorld::RemoveBody(Rigidbody* body) {
	if (body->invMass == 0.0f) {
		// Remove from static bodies vector
		const auto it = std::remove(staticBodies.begin(), staticBodies.end(), body);
		staticBodies.erase(it, staticBodies.end());
	}
	else {
		// Remove from dynamic bodies vector
		const auto it = std::remove(bodies.begin(), bodies.end(), body);
		bodies.erase(it, bodies.end());
	}
}