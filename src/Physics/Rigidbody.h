#pragma once
#include <SFML/System/Vector2.hpp>

class Rigidbody {
public:
	Rigidbody(float mass, float friction, float damping);
	float mass;
	float friction;
	float damping;
	sf::Vector2f velocity;
	sf::Vector2f position;

	void Step(float deltaTime);

	void ApplyForce(float force, sf::Vector2f direction);

	void ApplyDamping(float deltaTime);

	void ApplyGravity(sf::Vector2f gravity, float deltaTime);

	void StepPosition(float deltaTime);
};


// F = m * a
// V = V0 + a * t
// P1 = P0 + V * t
// J = ( -(1+e) * (V1*n)) / ( (1/m1) + (1/m2) )