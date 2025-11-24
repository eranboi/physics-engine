#include "Rigidbody.h"
#include <cmath>

Rigidbody::Rigidbody(const ShapeType shapeType, const RigidbodyConfig& config)
	: shapeType(shapeType), velocity({ 0,0 }), angularVelocity(0.0f),
	  mass(config.mass), friction(config.friction), damping(config.damping),
	  angularDamping(config.angularDamping), restitution(config.restitution),
	  radius(0.0f), inertia(0.0f), invInertia(0.0f), position({ 0,0 }), rotation(0.0f)
{
	if (mass > 0.0f) {
		invMass = 1.0f / mass;
	}
	else {
		invMass = 0.0f; // Infinite mass for static objects
	}
}

Rigidbody* Rigidbody::CreateCircle(const float radius, const RigidbodyConfig& config) {
	auto body = new Rigidbody(ShapeType::Circle, config);
	body->radius = radius;

	if (config.mass > 0.0f) {
		// Inertia for a solid circle: I = (1/2) * m * r^2
		body->inertia = 0.5f * config.mass * radius * radius;
		body->invInertia = 1.0f / body->inertia;
	}
	else {
		body->inertia = 0.0f;
		body->invInertia = 0.0f;
	}

	return body;
}

Rigidbody* Rigidbody::CreateBox(const float width, const float height, const RigidbodyConfig& config) {
	float halfW = width / 2.0f;
	float halfH = height / 2.0f;
	auto body = new Rigidbody(ShapeType::Polygon, config);

	// Changed the order of vertices to calculate the normals.
	// because y is inverted in SFML. (-y, x) used to give the inward direction.
	body->vertices.push_back(sf::Vector2f(-halfW, -halfH)); // Top Left
	body->vertices.push_back(sf::Vector2f(-halfW, halfH));  // Bottom Left
	body->vertices.push_back(sf::Vector2f(halfW, halfH));   // Bottom Right
	body->vertices.push_back(sf::Vector2f(halfW, -halfH));  // Top Right

	if (config.mass > 0.0f) {
		// Inertia for a rectangle: I = (1/12) * m * (w^2 + h^2)
		body->inertia = (1.0f / 12.0f) * config.mass * (width * width + height * height);
		body->invInertia = 1.0f / body->inertia;
	}
	else {
		body->inertia = 0.0f;
		body->invInertia = 0.0f;
	}

	return body;
}

Rigidbody* Rigidbody::CreateTriangle(const sf::Vector2f p1, const sf::Vector2f p2, const sf::Vector2f p3, const RigidbodyConfig& config) {
	auto body = new Rigidbody(ShapeType::Polygon, config);

	// Calculate centroid
	sf::Vector2f center = (p1 + p2 + p3) / 3.0f;
	body->position = center;

	// Store vertices relative to centroid
	body->vertices.push_back(p1 - center);
	body->vertices.push_back(p3 - center);
	body->vertices.push_back(p2 - center);

	if (config.mass > 0.0f) {
		// Use the generic polygon inertia calculation
		body->inertia = CalculatePolygonInertia(body->vertices, config.mass);
		body->invInertia = 1.0f / body->inertia;
	}
	else {
		body->inertia = 0.0f;
		body->invInertia = 0.0f;
	}

	return body;
}

Rigidbody* Rigidbody::CreatePolygon(const std::vector<sf::Vector2f>& vertices, const RigidbodyConfig& config) {
	if (vertices.size() < 3) {
		// Invalid polygon
		return nullptr;
	}

	auto body = new Rigidbody(ShapeType::Polygon, config);

	// Calculate centroid
	sf::Vector2f centroid(0.0f, 0.0f);
	for (const auto& v : vertices) {
		centroid += v;
	}
	centroid /= static_cast<float>(vertices.size());
	body->position = centroid;

	// Store vertices relative to centroid
	for (const auto& v : vertices) {
		body->vertices.push_back(v - centroid);
	}

	if (config.mass > 0.0f) {
		// Calculate inertia using the polygon method
		body->inertia = CalculatePolygonInertia(body->vertices, config.mass);
		body->invInertia = 1.0f / body->inertia;
	}
	else {
		body->inertia = 0.0f;
		body->invInertia = 0.0f;
	}

	return body;
}

float Rigidbody::CalculatePolygonInertia(const std::vector<sf::Vector2f>& vertices, float mass) {
	// Using the formula for moment of inertia of a polygon about its centroid
	// I = (mass / 6) * sum(|cross(v[i], v[i+1])| * (dot(v[i], v[i]) + dot(v[i], v[i+1]) + dot(v[i+1], v[i+1])))

	float numerator = 0.0f;
	float denominator = 0.0f;

	for (size_t i = 0; i < vertices.size(); i++) {
		const sf::Vector2f& v1 = vertices[i];
		const sf::Vector2f& v2 = vertices[(i + 1) % vertices.size()];

		float cross = std::abs(MathUtils::Cross(v1, v2));
		float dot1 = MathUtils::Dot(v1, v1);
		float dot2 = MathUtils::Dot(v1, v2);
		float dot3 = MathUtils::Dot(v2, v2);

		numerator += cross * (dot1 + dot2 + dot3);
		denominator += cross;
	}

	if (denominator < 0.0001f) {
		// Degenerate polygon, use approximation
		float maxDist = 0.0f;
		for (const auto& v : vertices) {
			float dist = std::sqrt(MathUtils::Dot(v, v));
			if (dist > maxDist) maxDist = dist;
		}
		return mass * maxDist * maxDist;
	}

	return (mass / 6.0f) * (numerator / denominator);
}

std::vector<sf::Vector2f> Rigidbody::GetTransformedVertices() const {
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

void Rigidbody::Step(const float deltaTime) {
	StepPosition(deltaTime);
	StepRotation(deltaTime);
	ApplyDamping(deltaTime);
	ApplyAngularDamping(deltaTime);
}

void Rigidbody::ApplyForce(const float force, const sf::Vector2f direction) {
	sf::Vector2f acc = (force * invMass) * direction;
	this->velocity += acc;
}

void Rigidbody::ApplyDamping(const float deltaTime) {
	this->velocity *= 1.0f / (1.0f + this->damping * deltaTime);
}

void Rigidbody::ApplyAngularDamping(const float deltaTime)
{
	this->angularVelocity *= 1.0f / (1.0f + this->angularDamping * deltaTime);
}

void Rigidbody::ApplyGravity(const sf::Vector2f gravity, const float deltaTime) {
	// Static bodies
	if (invMass == 0) return;
	this->velocity += gravity * deltaTime;
}

void Rigidbody::StepPosition(const float deltaTime) {
	this->position += this->velocity * deltaTime;
}

void Rigidbody::StepRotation(const float deltaTime)
{
	this->rotation += this->angularVelocity * deltaTime;
	while (this->rotation > 3.14f) this->rotation -= 2.0f * 3.14f;
	while (this->rotation < -3.14f) this->rotation += 2.0f * 3.14f;
}

void Rigidbody::OnDrawGizmos() const {
	return;

	std::vector<sf::Vector2f> currentVertices = GetTransformedVertices();

	for (int i = 0; i < currentVertices.size(); i++)
	{
		sf::Vector2f p1 = currentVertices[i];
		sf::Vector2f p2 = currentVertices[(i + 1) % currentVertices.size()];

		sf::Vector2f centerOfEdge = (p1 + p2) / 2.0f;
		sf::Vector2f edge = p2 - p1;
		auto normal = sf::Vector2f(-edge.y, edge.x);

		normal = MathUtils::Normalize(normal);

		// Draw vertices
		// Gizmos::DrawPoint(p1, sf::Color::White, 0.02f);
		// Gizmos::DrawText(p1, std::to_string(i));

		// Draw edges
		// Gizmos::DrawLine(p1, p2, sf::Color::Green);

		// Draw the normal
		float normalLength = 0.25f;
		sf::Vector2f arrowEnd = centerOfEdge + (normal * normalLength);

		/// Gizmos::DrawArrow(centerOfEdge, arrowEnd, sf::Color::White, 0.02f, 0.1f);
	}

	// Log the friction value
	char frictionText[16];
	snprintf(frictionText, sizeof(frictionText), "%.1f", friction);
	Gizmos::DrawText(this->position, frictionText);
}