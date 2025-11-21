#pragma once
#include <SFML/System/Vector2.hpp>
#include <SFML/Graphics/Color.hpp>
#include <vector>

enum ShapeType {
	Circle,
	Polygon
};

class Rigidbody {
public:
	static Rigidbody* CreateCircle(float radius, float mass, float restitution);
	static Rigidbody* CreateBox(float width, float height, float mass, float restitution);
	static Rigidbody* CreateTriangle(sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3, float mass, float restitution);

	// Physics
	ShapeType shapeType;
	float mass;
	float invMass;
	float friction;
	float damping;
	float restitution;
	float radius; // Only used for circles

	// Transform
	sf::Vector2f velocity;
	sf::Vector2f position;
	float rotation; // Radians

	// Rendering
	sf::Color color;

	void Step(float deltaTime);
	void ApplyForce(float force, sf::Vector2f direction);
	void ApplyDamping(float deltaTime);
	void ApplyGravity(sf::Vector2f gravity, float deltaTime);
	void StepPosition(float deltaTime);

	std::vector<sf::Vector2f> GetTransformedVertices();

private:
	Rigidbody(ShapeType shapeType, float mass, float friction, float damping, float restitution);	
	std::vector<sf::Vector2f> vertices;

};

