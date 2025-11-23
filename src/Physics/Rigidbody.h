#pragma once
#include <SFML/System/Vector2.hpp>
#include <SFML/Graphics/Color.hpp>
#include <vector>
#include "../Debugging/Gizmos.h"
#include <Utils/MathUtils.h>


enum ShapeType {
	Circle,
	Polygon
};

class Rigidbody {
public:
	static Rigidbody* CreateCircle(float radius, float mass, float restitution);
	static Rigidbody* CreateBox(float width, float height, float mass, float restitution);
	static Rigidbody* CreateTriangle(sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3, float mass, float restitution);

	void OnDrawGizmos();

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

	std::vector<sf::Vector2f> GetTransformedVertices();

private:
	Rigidbody(ShapeType shapeType, float mass, float friction, float damping, float restitution);	
	std::vector<sf::Vector2f> vertices;

};

