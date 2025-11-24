#pragma once
#include <SFML/System/Vector2.hpp>
#include <SFML/Graphics/Color.hpp>
#include <vector>
#include "../../Debugging/Gizmos.h"
#include <Utils/MathUtils.h>


enum ShapeType {
	Circle,
	Polygon
};

struct RigidbodyConfig {
	float mass = 1.0f;
	float restitution = 0.5f;
	float friction = 0.5f;
	float damping = 0.8f;
	float angularDamping = 2.0f;
};

class Rigidbody {
public:
	// Specialized shapes with proper inertia calculations
	static Rigidbody* CreateCircle(float radius, const RigidbodyConfig& config);
	static Rigidbody* CreateBox(float width, float height, const RigidbodyConfig& config);
	static Rigidbody* CreateTriangle(sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3, const RigidbodyConfig& config);

	// Generic polygon creator for custom shapes
	static Rigidbody* CreatePolygon(const std::vector<sf::Vector2f>& vertices, const RigidbodyConfig& config);

	void OnDrawGizmos() const;

	// Physics
	ShapeType shapeType;
	sf::Vector2f velocity;
	float angularVelocity; // Radians/s
	float mass;
	float invMass;
	float friction;
	float damping;
	float angularDamping;
	float restitution;
	float radius; // Only used for circles
	float inertia;
	float invInertia;

	// Transform
	sf::Vector2f position;
	float rotation; // Radians

	// Rendering
	sf::Color color;

	void Step(float deltaTime);
	void ApplyForce(float force, sf::Vector2f direction);
	void ApplyDamping(float deltaTime);
	void ApplyAngularDamping(float deltaTime);
	void ApplyGravity(sf::Vector2f gravity, float deltaTime);
	void StepPosition(float deltaTime);
	void StepRotation(float deltaTime);

	std::vector<sf::Vector2f> GetTransformedVertices() const;

private:
	Rigidbody(ShapeType shapeType, const RigidbodyConfig& config);
	std::vector<sf::Vector2f> vertices;

	// Helper function to calculate polygon inertia
	static float CalculatePolygonInertia(const std::vector<sf::Vector2f>& vertices, float mass);
};