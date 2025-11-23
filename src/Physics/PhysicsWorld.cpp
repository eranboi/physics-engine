#include "PhysicsWorld.h"
#include <cmath>
#include "./Collision/BroadPhase/Grid.h"
#include <SFML/System/Vector2.hpp>
#include "./Utils/MathUtils.h"
#include <algorithm>

constexpr int solverIterations = 8;

void PhysicsWorld::Step(const float deltaTime) {
	// Reset the grid
	grid.Clear();
	for (Rigidbody* body : bodies) {
		body->Step(deltaTime);
		body->ApplyGravity(gravity, deltaTime);

		// Fill the grid with bodies
		grid.AddBody(body);
	}

	for (int i = 0; i < solverIterations; i++)
	{
		ResolveCollisions();
	}
}

void PhysicsWorld::ResolveCollisions() const {
	for (Rigidbody* bodyA : bodies)
	{
		// Another collision check for the walls
		for (Rigidbody* wall : staticBodies) {
			CheckForCollision(*bodyA, *wall);
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
		const auto it = std::remove(staticBodies.begin(), staticBodies.end(), body);
		staticBodies.erase(it, staticBodies.end());
	}
	else {
		const auto it = std::remove(bodies.begin(), bodies.end(), body);
		bodies.erase(it, bodies.end());
	}
}

// SAT collision detection
void PhysicsWorld::CheckForCollision(Rigidbody& bodyA, Rigidbody& bodyB) {

	// Get the vertices of the bodies
	std::vector<sf::Vector2f> verticesA = bodyA.GetTransformedVertices();
	std::vector<sf::Vector2f> verticesB = bodyB.GetTransformedVertices();

	// Get the axes to test the shapes against. 
	std::vector<sf::Vector2f> axesA = GetAxes(verticesA);
	std::vector<sf::Vector2f> axesB = GetAxes(verticesB);

	// Combine the axes in a vector
	std::vector<AxisInfo> axes;

	for (int i = 0; i < axesA.size(); i++) {
		sf::Vector2f normal = axesA[i];
		axes.push_back({ normal, &bodyA, i });
	}

	for (int i = 0; i < axesB.size(); i++) {
		sf::Vector2f normal = axesB[i];
		axes.push_back({ normal, &bodyB, i });
	}

	AxisInfo bestAxis;
	float mtv = std::numeric_limits<float>::max();
	bool colliding = false;

	// Project all the vertices onto the axes. 
	// Get the min and max values for both shapes.
	// If we find an axis where they do NOT overlap, they're not colliding.
	for (auto const axis : axes) {

		float minA = std::numeric_limits<float>::max();
		float maxA = std::numeric_limits<float>::lowest();

		float minB = std::numeric_limits<float>::max();
		float maxB = std::numeric_limits<float>::lowest();

		for (auto vert : verticesA)
		{
			float dotProduct = MathUtils::Dot(vert, axis.normal);
			if (dotProduct <= minA) {
				minA = dotProduct;
			}
			if (dotProduct >= maxA)
			{
				maxA = dotProduct;
			}
		}

		for (auto vert : verticesB)
		{
			float dotProduct = MathUtils::Dot(vert, axis.normal);
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
			bestAxis = axis;
		}
	}
	if (!colliding) return;


	// Bodies
	Rigidbody* refBody = bestAxis.referenceBody;
	Rigidbody* incBody = (bestAxis.referenceBody == &bodyA) ? &bodyB : &bodyA;

	const auto& refVertices = refBody == &bodyA ? verticesA : verticesB;

	// find the collision normal (in which direction the inc body should move after collision)
	sf::Vector2f refToInc = incBody->position - refBody->position;

	// fix the collision normals direction. We need this correction because if two edges are parallel
	// like in a square, we might've selected the wrong edge.
	if (MathUtils::Dot(bestAxis.normal, refToInc) < 0) {
		bestAxis.normal = -bestAxis.normal;
	}

	float maxDot = std::numeric_limits<float>::lowest();

	// find the ref face here.
	for (int i = 0; i < refVertices.size(); i++)
	{
		sf::Vector2f p1 = refVertices[i];
		sf::Vector2f p2 = refVertices[(i + 1) % refVertices.size()];

		sf::Vector2f edge = p2 - p1;
		auto edgeNormal = sf::Vector2f(-edge.y, edge.x);

		float tempDot = MathUtils::Dot(edgeNormal, bestAxis.normal);

		if (tempDot > maxDot) {
			maxDot = tempDot;
			bestAxis.edgeIndex = i;
		}
	}
	
	// Grab the normals and vertices of the incident body
	// Normals are for comparison, vertices are for clipping
	std::vector<sf::Vector2f> incNormals = incBody == &bodyA ? axesA : axesB;
	const std::vector<sf::Vector2f>& incVertices = incBody == &bodyA ? verticesA : verticesB;

	int incFaceIndex = -1;
	float minDot = std::numeric_limits<float>::max();

	// Find the face that is most anti-parallel to the reference face normal
	// This is the face that is colliding
	for (int i = 0; i < incNormals.size(); i++)
	{
		float dot = MathUtils::Dot(bestAxis.normal, incNormals[i]);
		if (dot < minDot) {
			minDot = dot;
			incFaceIndex = i;
		}
	}

	// Get the incident face vertices to be clipped
	std::vector<sf::Vector2f> pointsOnIncFace{ incVertices[incFaceIndex] , incVertices[(incFaceIndex + 1) % incVertices.size()] };

	// Get the reference face vertices which act as the clipping boundaries
	int refIndex = bestAxis.edgeIndex;
	sf::Vector2f refV1 = refVertices[refIndex];
	sf::Vector2f refV2 = refVertices[(refIndex + 1) % refVertices.size()];

	// Find the edge direction
	sf::Vector2f edgeDir = refV2 - refV1;
	edgeDir = MathUtils::Normalize(edgeDir);

	// Do the clip
	std::vector<sf::Vector2f> clippedPoints = Clip(pointsOnIncFace, edgeDir, MathUtils::Dot(edgeDir, refV1));
	if (clippedPoints.size() < 2) return;
	clippedPoints = Clip(clippedPoints, -edgeDir, MathUtils::Dot(-edgeDir, refV2));

	// Find the exact point to compare the points against for the depth check 
	// Are the points inside the ref body?
	float refOffset = MathUtils::Dot(bestAxis.normal, refV1);

	// Craete the collision manifold.
	auto manifold = CollisionManifold();

	// Check every point against the face with dot product.
	for (sf::Vector2f p : clippedPoints) {
		float depth = MathUtils::Dot(bestAxis.normal, p) - refOffset;

		if (depth <= 0.0f) {
			manifold.contactPoints.push_back(p);
		}
	}

	if (manifold.contactPoints.empty()) {
		return;
	}


	manifold.refBody = refBody;
	manifold.incBody = incBody;
	manifold.depth = mtv;
	manifold.collisionAxis = bestAxis.normal;

	// Resolve the collision using the calculated MTV and axis
	ResolveCollision(manifold);
}

// Basic circle-circle collision detection
void PhysicsWorld::CheckForCircleCircleCollision(const Rigidbody* bodyA, const Rigidbody* bodyB) {
	const sf::Vector2f collisionNormal = bodyB->position - bodyA->position;

	// Find the distance between bodies using Euclidean Distance 
	// Optimization: use distance squared to avoid sqrt until necessary
	const float distanceSq = collisionNormal.x * collisionNormal.x + collisionNormal.y * collisionNormal.y;
	const float totalRadius = bodyA->radius + bodyB->radius;
	const float totalRadiusSq = totalRadius * totalRadius;

	// If the distance is smaller than the total radius, they're colliding.
	// If they're colliding, find the MTV and move the bodies accordingly.
	if (distanceSq < totalRadiusSq) {

		// Calculate actual distance
		const float distance = std::sqrt(distanceSq);

		float penetrationDepth = totalRadius - distance;

		// sf::Vector2f collisionNormalNormalized = collisionNormal / distance;

		// ResolveCollision(*bodyA, *bodyB, penetrationDepth, collisionNormal);
	}
}

void PhysicsWorld::ResolveCollision(const CollisionManifold& manifold) {

	const sf::Vector2f normal = MathUtils::Normalize(manifold.collisionAxis);
	Rigidbody& refBody = *manifold.refBody;
	Rigidbody& incBody = *manifold.incBody;
	const float depth = manifold.depth;



	// Positional correction based on mass
	const float totalInvMass = refBody.invMass + incBody.invMass;
	if (totalInvMass > 0.0f)
	{
		// Baumgarte stabilization
		// Don't try to apply all the positional correction in one step.
		constexpr float slop = 0.01f;
		constexpr float percent = 0.2f;
		const float correctionDepth = std::max(depth - slop, 0.0f);
		const sf::Vector2f correction = normal * (correctionDepth / totalInvMass) * percent;
		refBody.position -= correction * refBody.invMass;
		incBody.position += correction * incBody.invMass;
	}

	// If we don't have contact points, we can't calculate rotation.
	// We only did positional correction above to prevent sinking.
	if (manifold.contactPoints.empty()) return;

	// Calculate the contact point.
	// we average all contact points for stability.
	auto contactPoint = sf::Vector2f(0, 0);
	for (const auto& p : manifold.contactPoints) {
		contactPoint += p;
		Gizmos::DrawPoint(p);
	}
	contactPoint /= static_cast<float>(manifold.contactPoints.size());

	// Calculate the moment arm
	const sf::Vector2f rA = contactPoint - refBody.position;
	const sf::Vector2f rB = contactPoint - incBody.position;

	// Calculate Relative Velocity
	// V_rel = V_b - V_a
	// But we must include the rotational velocity at that specific point:
	// V_point = V_linear + (AngularVelocity * Radius_Perpendicular)
	const sf::Vector2f velA = refBody.velocity + MathUtils::Cross(refBody.angularVelocity, rA);
	const sf::Vector2f velB = incBody.velocity + MathUtils::Cross(incBody.angularVelocity, rB);

	// Find the relvative velocity
	const sf::Vector2f relativeVelocity = velB - velA;

	// Find the velocity along the normal of the collision, using dot product.
	const float velocityAlongNormal = MathUtils::Dot(relativeVelocity, normal);

	// Early Exit: If bodies are already separating, don't apply impulse.
	if (velocityAlongNormal > 0) return;

	// Get the minimum restitution of the two bodies
	const float e = std::min(refBody.restitution, incBody.restitution);

	// Calculate the Impulse Scalar (j)
	// Denominator terms:
	// 1. Linear Mass (invMass)
	// 2. Rotational Inertia ((r x n)^2 * invInertia) -> This adds resistance to rotation.

	const float raCrossN = MathUtils::Cross(rA, normal);
	const float rbCrossN = MathUtils::Cross(rB, normal);

	const float invMassSum = refBody.invMass + incBody.invMass +
		(raCrossN * raCrossN) * refBody.invInertia +
		(rbCrossN * rbCrossN) * incBody.invInertia;

	// Apply the impulse formula: j = -(1 + e) * V_rel / TotalMassAndInertia
	const float j = (-(1.0f + e) * velocityAlongNormal) / invMassSum;

	// Calculate the final Impulse Vector
	const sf::Vector2f impulse = j * normal;

	const float torqueA = MathUtils::Cross(rA, impulse);
	const float torqueB = MathUtils::Cross(rB, impulse);

	// Apply linear impulse
	refBody.velocity -= impulse * refBody.invMass;
	incBody.velocity += impulse * incBody.invMass;

	// Apply angular impulse 
	refBody.angularVelocity -= torqueA * refBody.invInertia;
	incBody.angularVelocity += torqueB * incBody.invInertia;
}

std::vector<sf::Vector2f> PhysicsWorld::GetAxes(const std::vector<sf::Vector2f>& vertices) {
	std::vector<sf::Vector2f> axes;

	// Find the axis by getting the vertices.
	for (int i = 0; i < vertices.size(); i++) {
		const sf::Vector2f p1 = vertices[i];
		const sf::Vector2f p2 = vertices[(i + 1) % vertices.size()];

		// calculate the vector representing the edge using the two vertices
		const sf::Vector2f edge = p2 - p1;

		// calculate the perpendicular vector (90 degrees anti-clockwise) 
		// if we want to do it clockwise we can simply do (edge.y, -edge.x)
		// in our case it does not matter.
		auto edgeNormal = sf::Vector2f(-edge.y, edge.x);

		// normalize the perpendecular
		sf::Vector2f normalNormalized = MathUtils::Normalize(edgeNormal);

		axes.push_back(normalNormalized);
	}

	return axes;
}

// Clip the line (used to get the contact points.) No depth check here. 
std::vector<sf::Vector2f> PhysicsWorld::Clip(const std::vector<sf::Vector2f>& incFace, const sf::Vector2f& normal, const float offset) {


	std::vector<sf::Vector2f> contactPoints;

	// we need at 2 points for it to considered as a face.
	if (incFace.size() < 2) return contactPoints;

	// Get the points
	const sf::Vector2f v1 = incFace[0];
	const sf::Vector2f v2 = incFace[1];

	//std::cout << "--- Clip Debug ---" << std::endl;
	//std::cout << "Normal: " << normal.x << ", " << normal.y << " | Offset: " << offset << std::endl;
	//std::cout << "V1 Pos: (" << v1.x << ", " << v1.y << ")" << std::endl;
	//std::cout << "V2 Pos: (" << v2.x << ", " << v2.y << ")" << std::endl;

	// get the dot product of the normal and points to see and then add the offset to see if the point is inside.
	const float d1 = MathUtils::Dot(normal, v1) - offset;
	const float d2 = MathUtils::Dot(normal, v2) - offset;
	//std::cout << "d1 is: " << d1 << std::endl;
	//std::cout << "d2 is: " << d2 << std::endl;

	// if dot value is bigger than 0 the point is inside the tunnel we created.
	if (d1 > 0.0f) {
		contactPoints.push_back(v1);
	}

	if (d2 > 0.0f) {
		contactPoints.push_back(v2);
	}

	// check if any of the point is left outside
	if (d1 * d2 < 0.0f) {
		// we'll find out how much outside the point is.
		// Think of this like d1 is +5 and d2 is -5 (meaning d1 is inside 5 meter and d2 is outside by 5 meters).
		// t is lerping factor. so 5 / 10 = 0.5 that means we need to slide that point by half of the distance of the face
		const float t = d1 / (d1 - d2);

		// get the intersection by lerping that much on the v1
		const sf::Vector2f intersectionPoint = v1 + (v2 - v1) * t;

		contactPoints.push_back(intersectionPoint);
	}

	return contactPoints;

}