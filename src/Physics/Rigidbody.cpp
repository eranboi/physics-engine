#include "Rigidbody.h"
#include <cmath>

Rigidbody::Rigidbody(ShapeType type, float mass, float friction, float damping, float restitution)
	: shapeType(type), mass(mass), friction(friction), damping(damping), restitution(restitution),
	velocity({ 0,0 }), position({ 0,0 }), rotation(0.0f), radius(0.0f),
	inertia(0.0f), invInertia(0.0f), angularVelocity(0.0f), angularDamping(2.0f)
{
	if (mass > 0.0f) {
		invMass = 1.0f / mass;
	}
	else {
		invMass = 0.0f; // Infinite mass for static objects
	}
}

Rigidbody* Rigidbody::CreateCircle(float radius, float mass, float restitution) {
	Rigidbody* body = new Rigidbody(ShapeType::Circle, mass, 0.5f, 0.8f, restitution);
	body->radius = radius;

	if (mass > 0.0f) {
		body->inertia = 0.5f * mass * radius * radius;
		body->invInertia = 1.0f / body->inertia;
	}
	else {
		body->inertia = 0.0f;
		body->invInertia = 0.0f;
	}

	return body;
}

Rigidbody* Rigidbody::CreateBox(float width, float height, float mass, float restitution) {
	float halfW = width / 2.0f;
	float halfH = height / 2.0f;
	Rigidbody* body = new Rigidbody(ShapeType::Polygon, mass, 0.5f, 0.8f, restitution);

	// Changed the order of vertices to calculate the normals.
	// because y is inverted in SFML. (-y, x) used to give the inward direction.
	body->vertices.push_back(sf::Vector2f(-halfW, -halfH)); // Top Left
	body->vertices.push_back(sf::Vector2f(-halfW, halfH));  // Bottom Left
	body->vertices.push_back(sf::Vector2f(halfW, halfH));   // Bottom Right
	body->vertices.push_back(sf::Vector2f(halfW, -halfH));  // Top Right

	if (mass > 0.0f) {
		body->inertia = (1.0f / 12.0f) * mass * (width * width + height * height);
		body->invInertia = 1.0f / body->inertia;
	}
	else {
		body->inertia = 0.0f;
		body->invInertia = 0.0f;
	}

	return body;
}

Rigidbody* Rigidbody::CreateTriangle(sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3, float mass, float restitution) {
	Rigidbody* body = new Rigidbody(ShapeType::Polygon, mass, 0.5f, 0.8f, restitution);
	sf::Vector2f center = (p1 + p2 + p3) / 3.0f;

	body->position = center;

	body->vertices.push_back(p1 - center);
	body->vertices.push_back(p3 - center);
	body->vertices.push_back(p2 - center);

	if (mass > 0.0f) {
		// find an approx value for width and height for testing
		float width = std::abs(p2.x - p1.x);
		float height = std::abs(p3.y - p1.y);

		// Calculate the inertia
		body->inertia = (1.0f / 12.0f) * mass * (width * width + height * height);
		body->invInertia = 1.0f / body->inertia;
	}
	else {
		body->inertia = 0.0f;
		body->invInertia = 0.0f;
	}

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
	StepPosition(deltaTime);
	StepRotation(deltaTime);
	ApplyDamping(deltaTime);
	ApplyAngularDamping(deltaTime);
}

void Rigidbody::ApplyForce(float force, sf::Vector2f direction) {
	sf::Vector2f acc = (force * invMass) * direction;
	this->velocity += acc;
}

void Rigidbody::ApplyDamping(float deltaTime) {
	this->velocity *= 1.0f / (1.0f + this->damping * deltaTime);
}

void Rigidbody::ApplyAngularDamping(float deltaTime)
{
	this->angularVelocity *= 1.0f / (1.0f + this->angularDamping * deltaTime);
}

void Rigidbody::ApplyGravity(sf::Vector2f gravity, float deltaTime) {
	// Static bodies
	if (invMass == 0) return;
	this->velocity += gravity * deltaTime;
}

void Rigidbody::StepPosition(float deltaTime) {
	this->position += this->velocity * deltaTime;
}

void Rigidbody::StepRotation(float deltaTime)
{
	this->rotation += this->angularVelocity * deltaTime;
	while (this->rotation > 3.14f) this->rotation -= 2.0f * 3.14f;
	while (this->rotation < -3.14f) this->rotation += 2.0f * 3.14f;
}

void Rigidbody::OnDrawGizmos() {
	std::vector<sf::Vector2f> currentVertices = GetTransformedVertices();

	for (int i = 0; i < currentVertices.size(); i++)
	{
		sf::Vector2f p1 = currentVertices[i];
		sf::Vector2f p2 = currentVertices[(i + 1) % currentVertices.size()];

		sf::Vector2f centerOfEdge = (p1 + p2) / 2.0f;
		sf::Vector2f edge = p2 - p1;
		sf::Vector2f normal = sf::Vector2f(-edge.y, edge.x);

		normal = MathUtils::Normalize(normal);

		// Draw vertices
		// Gizmos::DrawPoint(p1, sf::Color::Red, 0.02f);
		// Gizmos::DrawText(p1, "p" + std::to_string(i));

		// Draw edges
		// Gizmos::DrawLine(p1, p2, sf::Color::Green);

		// Draw the normal
		float normalLength = 0.25f;
		sf::Vector2f arrowEnd = centerOfEdge + (normal * normalLength);

		// Gizmos::DrawArrow(centerOfEdge, arrowEnd, sf::Color::Yellow, 0.02f, 0.1f);
	}
}