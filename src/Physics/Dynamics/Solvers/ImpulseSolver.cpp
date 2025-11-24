#include "ImpulseSolver.h"

#include <iostream>

void ImpulseSolver::Solve(std::vector<CollisionManifold> &manifolds, float dt) {
	constexpr int solverIterations = 6;

	for (int i = 0; i < solverIterations; i++) {
		for (auto& manifold : manifolds) {
			if (i == 0) {
				ApplyPositionalCorrection(manifold);
			}
			ResolveVelocity(manifold);
		}
	}
}

void ImpulseSolver::ApplyPositionalCorrection(const CollisionManifold &manifold) {
	const sf::Vector2f normal = MathUtils::Normalize(manifold.collisionAxis);
	Rigidbody* refBody = manifold.refBody;
	Rigidbody* incBody = manifold.incBody;
	const float depth = manifold.depth;

	// Positional correction based on mass
	const float totalInvMass = refBody->invMass + incBody->invMass;
	if (totalInvMass > 0.0f) {
		constexpr float slop = 0.01f;
		constexpr float percent = 0.2f;
		constexpr float maxCorrection = 0.2f;

		float correctionDepth = std::max(depth - slop, 0.0f);
		correctionDepth = std::min(correctionDepth, maxCorrection); // LIMIT

		const sf::Vector2f correction = normal * (correctionDepth / totalInvMass) * percent;
		refBody->position -= correction * refBody->invMass;
		incBody->position += correction * incBody->invMass;
	}
}

void ImpulseSolver::ResolveVelocity(const CollisionManifold &manifold) {
	const sf::Vector2f normal = MathUtils::Normalize(manifold.collisionAxis);
	Rigidbody* refBody = manifold.refBody;
	Rigidbody* incBody = manifold.incBody;

	// If we don't have contact points, we can't calculate rotation.
	// We only did positional correction above to prevent sinking.
	if (manifold.contactPoints.empty()) return;

	// Calculate the contact point.
	// we average all contact points for stability.
	auto contactPoint = sf::Vector2f(0, 0);
	for (const auto& p : manifold.contactPoints) {
		contactPoint += p;
		Gizmos::DrawCross(p, 0.05f, sf::Color::Yellow);
	}
	contactPoint /= static_cast<float>(manifold.contactPoints.size());

	// Calculate the moment arm
	const sf::Vector2f rA = contactPoint - refBody->position;
	const sf::Vector2f rB = contactPoint - incBody->position;

	// Calculate Relative Velocity
	// V_rel = V_b - V_a
	// But we must include the rotational velocity at that specific point:
	// V_point = V_linear + (AngularVelocity * Radius_Perpendicular)
	const sf::Vector2f velA = refBody->velocity + MathUtils::Cross(refBody->angularVelocity, rA);
	const sf::Vector2f velB = incBody->velocity + MathUtils::Cross(incBody->angularVelocity, rB);

	// Find the relative velocity
	const sf::Vector2f relativeVelocity = velB - velA;

	// Find the velocity along the normal of the collision, using dot product.
	const float velocityAlongNormal = MathUtils::Dot(relativeVelocity, normal);

	// Early Exit: If bodies are already separating, don't apply impulse.
	if (velocityAlongNormal > 0) return;

	// Get the minimum restitution of the two bodies
	const float e = std::min(refBody->restitution, incBody->restitution);

	// Calculate the Impulse Scalar (j)
	// Denominator terms:
	// 1. Linear Mass (invMass)
	// 2. Rotational Inertia ((r x n)^2 * invInertia) -> This adds resistance to rotation.

	const float raCrossN = MathUtils::Cross(rA, normal);
	const float rbCrossN = MathUtils::Cross(rB, normal);

	const float invMassSum = refBody->invMass + incBody->invMass +
		(raCrossN * raCrossN) * refBody->invInertia +
		(rbCrossN * rbCrossN) * incBody->invInertia;

	// Apply the impulse formula: j = -(1 + e) * V_rel / TotalMassAndInertia
	const float j = (-(1.0f + e) * velocityAlongNormal) / invMassSum;

	// Calculate the final Impulse Vector
	const sf::Vector2f impulse = j * normal;

	const float torqueA = MathUtils::Cross(rA, impulse);
	const float torqueB = MathUtils::Cross(rB, impulse);

	// Apply linear impulse
	refBody->velocity -= impulse * refBody->invMass;
	incBody->velocity += impulse * incBody->invMass;

	// Apply angular impulse
	refBody->angularVelocity -= torqueA * refBody->invInertia;
	incBody->angularVelocity += torqueB * incBody->invInertia;
}

