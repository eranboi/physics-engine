#include "Rigidbody.h"

Rigidbody::Rigidbody(float mass, float friction, float damping)
	: mass(mass), friction(friction), damping(damping), velocity(sf::Vector2f(0.f, 0.f)), position(sf::Vector2f(0.f, 0.f)) {
}

void Rigidbody::Step(float deltaTime) {
	ApplyGravity(sf::Vector2f(0.0f, 9.81f), deltaTime);
	// ApplyDamping(deltaTime);
	StepPosition(deltaTime);
}

void Rigidbody::ApplyForce(float force, sf::Vector2f direction) {
	sf::Vector2f acc = (force / mass) * direction;
	this->velocity += acc;
}
void Rigidbody::ApplyDamping(float deltaTime) {
	this->velocity *= this->damping;
}
void Rigidbody::ApplyGravity(sf::Vector2f gravity, float deltaTime) {
	sf::Vector2f acc = gravity;
	this->velocity += acc * deltaTime;
}
void Rigidbody::StepPosition(float deltaTime) {
	this->position += this->velocity * deltaTime;
}
