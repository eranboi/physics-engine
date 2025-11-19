#include "Rigidbody.h"

Rigidbody::Rigidbody(float mass, float friction, float damping, float radius, float restitution)
	: mass(mass), invMass(1/mass), friction(friction), damping(damping), radius(radius), restitution(restitution),
	velocity(sf::Vector2f(0.f, 0.f)){
}

void Rigidbody::Step(float deltaTime) {
	ApplyDamping();
}

void Rigidbody::ApplyForce(float force, sf::Vector2f direction) {
	sf::Vector2f acc = (force / mass) * direction;
	this->velocity += acc;
}
void Rigidbody::ApplyDamping() {
	this->velocity *= this->damping;
}
void Rigidbody::ApplyGravity(sf::Vector2f gravity, float deltaTime) {
	sf::Vector2f acc = gravity;
	this->velocity += acc * deltaTime;
}
void Rigidbody::StepPosition(float deltaTime) {
	this->position += this->velocity * deltaTime;
}
