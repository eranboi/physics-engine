#pragma once
#include <SFML/System/Vector2.hpp>
#include <SFML/Graphics/Color.hpp>

class Rigidbody {
public:
	Rigidbody(float mass, float friction, float damping, float radius, float restitution);
	float mass;
	float invMass;
	float friction;
	float damping;
	float radius;
	float restitution;
	sf::Color color;

	sf::Vector2f velocity;
	sf::Vector2f position;

	void Step(float deltaTime);

	void ApplyForce(float force, sf::Vector2f direction);

	void ApplyDamping();

	void ApplyGravity(sf::Vector2f gravity, float deltaTime);

	void StepPosition(float deltaTime);
};