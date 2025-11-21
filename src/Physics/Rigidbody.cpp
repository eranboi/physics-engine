#include "Rigidbody.h"
#include <cmath>

Rigidbody::Rigidbody(ShapeType type, float mass, float friction, float damping, float restitution)
	: shapeType(type), mass(mass), friction(friction), damping(damping), restitution(restitution),
	velocity({ 0,0 }), position({ 0,0 }), rotation(0.0f), radius(0.0f)
{
	if (mass > 0.0f) {
		invMass = 1.0f / mass;
	}
	else {
		invMass = 0.0f; // Infinite mass for static objects
	}
}

Rigidbody* Rigidbody::CreateCircle(float radius, float mass, float restitution) {
	Rigidbody* body = new Rigidbody(ShapeType::Circle, mass, 0.5f, 0.5f, restitution);
	body->radius = radius;
	return body;
}

Rigidbody* Rigidbody::CreateBox(float width, float height, float mass, float restitution) {
	float halfW = width / 2.0f;
	float halfH = height / 2.0f;
	Rigidbody* body = new Rigidbody(ShapeType::Polygon, mass, 0.5f, 0.5f, restitution);

	body->vertices.push_back(sf::Vector2f(-halfW, -halfH));
	body->vertices.push_back(sf::Vector2f(halfW, -halfH));
	body->vertices.push_back(sf::Vector2f(halfW, halfH));
	body->vertices.push_back(sf::Vector2f(-halfW, halfH));

	return body;
}

Rigidbody* Rigidbody::CreateTriangle(sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3, float mass, float restitution) {
	Rigidbody* body = new Rigidbody(ShapeType::Polygon, mass, 0.5f, 0.5f, restitution);
	sf::Vector2f center = (p1 + p2 + p3) / 3.0f;

	body->position = center;

	body->vertices.push_back(p1 - center);
	body->vertices.push_back(p2 - center);
	body->vertices.push_back(p3 - center);

	return body;
}

std::vector<sf::Vector2f> Rigidbody::GetTransformedVertices() {
	// Circle has no vertices
	if (shapeType == ShapeType::Circle) {
		return {};
	}

	std::vector<sf::Vector2f> transformedVertices;

	float cos = std::cos(rotation);
	float sin = std::sin(rotation);

	// rx = cos * x - sin * y
	// ry = sin * x + cos * y

	for (const auto& v : vertices) {
		// calculate the vertices position based on rotation
		float rx = (v.x * cos) - (v.y * sin);
		float ry = (sin * v.x) + (cos * v.y);

		// add the body position to get the actual world position of the vertices
		sf::Vector2f finalPos;
		finalPos.x = rx + position.x;
		finalPos.y = ry + position.y;

		transformedVertices.push_back(finalPos);
	}

	return transformedVertices;


}

void Rigidbody::Step(float deltaTime) {
	ApplyDamping(deltaTime);
}

void Rigidbody::ApplyForce(float force, sf::Vector2f direction) {
	sf::Vector2f acc = (force / mass) * direction;
	this->velocity += acc;
}

void Rigidbody::ApplyDamping(float deltaTime) {
	this->velocity *= 1.0f / (1.0f + this->damping * deltaTime);
}

void Rigidbody::ApplyGravity(sf::Vector2f gravity, float deltaTime) {
	// Static bodies
	if (invMass == 0) return;
	this->velocity += gravity * deltaTime;
}

void Rigidbody::StepPosition(float deltaTime) {
	this->position += this->velocity * deltaTime;
}

