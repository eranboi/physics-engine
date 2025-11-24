#include "CollisionDetector.h"

bool CollisionDetector::CheckForCollision(Rigidbody* bodyA, Rigidbody* bodyB, CollisionManifold& outManifold) {
	// Determine shape types
	const bool isCircleA = bodyA->shapeType == ShapeType::Circle;
	const bool isCircleB = bodyB->shapeType == ShapeType::Circle;

	// Circle vs Circle
	if (isCircleA && isCircleB) {
		return CircleVsCircle(bodyA, bodyB, outManifold);
	}

	// Polygon vs Polygon
	if (!isCircleA && !isCircleB) {
		return PolygonVsPolygon(bodyA, bodyB, outManifold);
	}

	// Polygon vs Circle
	if (isCircleA) {
		// bodyA is circle, bodyB is polygon
		return PolygonVsCircle(bodyB, bodyA, outManifold);
	} else {
		// bodyA is polygon, bodyB is circle
		return PolygonVsCircle(bodyA, bodyB, outManifold);
	}
}

bool CollisionDetector::PolygonVsPolygon(Rigidbody* bodyA, Rigidbody* bodyB, CollisionManifold& outManifold) {

	// Get the vertices of the bodies
	std::vector<sf::Vector2f> verticesA = bodyA->GetTransformedVertices();
	std::vector<sf::Vector2f> verticesB = bodyB->GetTransformedVertices();

	// Get the axes to test the polygons against.
	std::vector<sf::Vector2f> axesA = GetEdgeNormals(verticesA);
	std::vector<sf::Vector2f> axesB = GetEdgeNormals(verticesB);

	// Combine the axes in a vector
	std::vector<AxisInfo> axes;

	for (int i = 0; i < axesA.size(); i++) {
		sf::Vector2f normal = axesA[i];
		axes.push_back({ normal, bodyA, i });
	}

	for (int i = 0; i < axesB.size(); i++) {
		sf::Vector2f normal = axesB[i];
		axes.push_back({ normal, bodyB, i });
	}

	AxisInfo bestAxis;
	float mtv = std::numeric_limits<float>::max();
	bool colliding = false;

	// Project all the vertices onto the axes.
	// Get the min and max values for both shapes.
	// If we find an axis where they do NOT overlap, they're not colliding.
	for (auto const axis : axes) {

		float minA = std::numeric_limits<float>::max();
		float maxA = std::numeric_limits<float>::lowest();

		float minB = std::numeric_limits<float>::max();
		float maxB = std::numeric_limits<float>::lowest();

		for (auto vert : verticesA)
		{
			float dotProduct = MathUtils::Dot(vert, axis.normal);
			if (dotProduct <= minA) {
				minA = dotProduct;
			}
			if (dotProduct >= maxA)
			{
				maxA = dotProduct;
			}
		}

		for (auto vert : verticesB)
		{
			float dotProduct = MathUtils::Dot(vert, axis.normal);
			if (dotProduct <= minB) {
				minB = dotProduct;
			}
			if (dotProduct >= maxB)
			{
				maxB = dotProduct;
			}
		}

		// if the min value of something is bigger than the other
		// that means that is the seperating axis. No collision.
		if (minA > maxB || minB > maxA) {
			colliding = false;
			break;
		}

		colliding = true;

		// To find the overlap, we should get the minimum max value and maximum min value
		// then we subtract the maxMin from minMax
		float minMax = std::min(maxA, maxB);
		float maxMin = std::max(minA, minB);

		float tempMtv = minMax - maxMin;

		if (tempMtv < mtv)
		{
			mtv = tempMtv;
			bestAxis = axis;
		}
	}
	if (!colliding) return false;


	// Bodies
	Rigidbody* refBody = bestAxis.referenceBody;
	Rigidbody* incBody = (bestAxis.referenceBody == bodyA) ? bodyB : bodyA;

	const auto& refVertices = refBody == bodyA ? verticesA : verticesB;

	// find the collision normal (in which direction the inc body should move after collision)
	sf::Vector2f refToInc = incBody->position - refBody->position;

	// fix the collision normals direction. We need this correction because if two edges are parallel
	// like in a square, we might've selected the wrong edge.
	if (MathUtils::Dot(bestAxis.normal, refToInc) < 0) {
		bestAxis.normal = -bestAxis.normal;
	}

	float maxDot = std::numeric_limits<float>::lowest();

	// find the ref face here.
	for (int i = 0; i < refVertices.size(); i++)
	{
		sf::Vector2f p1 = refVertices[i];
		sf::Vector2f p2 = refVertices[(i + 1) % refVertices.size()];

		sf::Vector2f edge = p2 - p1;
		auto edgeNormal = sf::Vector2f(-edge.y, edge.x);

		float tempDot = MathUtils::Dot(edgeNormal, bestAxis.normal);

		if (tempDot > maxDot) {
			maxDot = tempDot;
			bestAxis.edgeIndex = i;
		}
	}

	// Grab the normals and vertices of the incident body
	// Normals are for comparison, vertices are for clipping
	std::vector<sf::Vector2f> incNormals = incBody == bodyA ? axesA : axesB;
	const std::vector<sf::Vector2f>& incVertices = incBody == bodyA ? verticesA : verticesB;

	int incFaceIndex = -1;
	float minDot = std::numeric_limits<float>::max();

	// Find the face that is most anti-parallel to the reference face normal
	// This is the face that is colliding
	for (int i = 0; i < incNormals.size(); i++)
	{
		float dot = MathUtils::Dot(bestAxis.normal, incNormals[i]);
		if (dot < minDot) {
			minDot = dot;
			incFaceIndex = i;
		}
	}

	// Get the incident face vertices to be clipped
	std::vector<sf::Vector2f> pointsOnIncFace{ incVertices[incFaceIndex] , incVertices[(incFaceIndex + 1) % incVertices.size()] };

	// Get the reference face vertices which act as the clipping boundaries
	int refIndex = bestAxis.edgeIndex;
	sf::Vector2f refV1 = refVertices[refIndex];
	sf::Vector2f refV2 = refVertices[(refIndex + 1) % refVertices.size()];

	// Find the edge direction
	sf::Vector2f edgeDir = refV2 - refV1;
	edgeDir = MathUtils::Normalize(edgeDir);

	// Do the clip
	std::vector<sf::Vector2f> clippedPoints = Clip(pointsOnIncFace, edgeDir, MathUtils::Dot(edgeDir, refV1));
	if (clippedPoints.size() < 2) return false;
	clippedPoints = Clip(clippedPoints, -edgeDir, MathUtils::Dot(-edgeDir, refV2));

	// Find the exact point to compare the points against for the depth check
	// Are the points inside the ref body?
	float refOffset = MathUtils::Dot(bestAxis.normal, refV1);


	// Check every point against the face with dot product.
	for (sf::Vector2f p : clippedPoints) {
		float depth = MathUtils::Dot(bestAxis.normal, p) - refOffset;

		if (depth <= 0.0f) {
			outManifold.contactPoints.push_back(p);
		}
	}

	if (outManifold.contactPoints.empty()) {
		return false;
	}

	outManifold.refBody = refBody;
	outManifold.incBody = incBody;
	outManifold.depth = mtv;
	outManifold.collisionAxis = bestAxis.normal;

	return true;
}

bool CollisionDetector::PolygonVsCircle(Rigidbody* polygon, Rigidbody* circle, CollisionManifold& outManifold) {
    const sf::Vector2f circleCenter = circle->position;
    const float circleRadius = circle->radius;
    
    // Get transformed vertices of the polygon
    const std::vector<sf::Vector2f> vertices = polygon->GetTransformedVertices();
    
    float minDistance = std::numeric_limits<float>::max();
    sf::Vector2f closestPoint;
    sf::Vector2f collisionAxis;
    
    // Find the closest edge of the polygon to the circle center
    for (size_t i = 0; i < vertices.size(); i++) {
        const sf::Vector2f v1 = vertices[i];
        const sf::Vector2f v2 = vertices[(i + 1) % vertices.size()];
        
        // Project circle center onto the edge
        sf::Vector2f edge = v2 - v1;
        sf::Vector2f toCircle = circleCenter - v1;
        
        float edgeLengthSquared = edge.x * edge.x + edge.y * edge.y;
        float t = std::max(0.0f, std::min(1.0f, (toCircle.x * edge.x + toCircle.y * edge.y) / edgeLengthSquared));
        
        // Closest point on this edge
        sf::Vector2f pointOnEdge = v1 + edge * t;
        sf::Vector2f toCircleFromEdge = circleCenter - pointOnEdge;
        float distanceSquared = toCircleFromEdge.x * toCircleFromEdge.x + toCircleFromEdge.y * toCircleFromEdge.y;
        
        if (distanceSquared < minDistance * minDistance) {
            minDistance = std::sqrt(distanceSquared);
            closestPoint = pointOnEdge;
            
            // Collision axis points from edge to circle
            if (minDistance > 0.0001f) {
                collisionAxis = toCircleFromEdge / minDistance;
            } else {
                // Circle center is on the edge, use edge normal
                sf::Vector2f edgeNormal(-edge.y, edge.x);
                float length = std::sqrt(edgeNormal.x * edgeNormal.x + edgeNormal.y * edgeNormal.y);
                collisionAxis = edgeNormal / length;
            }
        }
    }
    
    // Check if circle is colliding with polygon
    if (minDistance >= circleRadius) {
        return false;
    }
    
    // Calculate penetration depth
    const float depth = circleRadius - minDistance;
    
    // Fill manifold
    outManifold.refBody = polygon;
    outManifold.incBody = circle;
    outManifold.collisionAxis = collisionAxis;
    outManifold.depth = depth;
    outManifold.contactPoints.clear();
    outManifold.contactPoints.push_back(closestPoint);
    
    return true;
}

bool CollisionDetector::CircleVsCircle(Rigidbody* bodyA, Rigidbody* bodyB, CollisionManifold& outManifold) {
    const sf::Vector2f centerA = bodyA->position;
    const sf::Vector2f centerB = bodyB->position;
    
    const float radiusA = bodyA->radius;
    const float radiusB = bodyB->radius;
    
    // Calculate distance between centers
    const sf::Vector2f delta = centerB - centerA;
    const float distanceSquared = delta.x * delta.x + delta.y * delta.y;
    const float radiusSum = radiusA + radiusB;
    
    // Check if circles are colliding
    if (distanceSquared >= radiusSum * radiusSum) {
        return false;
    }
    
    const float distance = std::sqrt(distanceSquared);
    
    // Calculate collision axis (normal)
    sf::Vector2f collisionAxis;
    if (distance < 0.0001f) {
        // Circles are exactly on top of each other
        collisionAxis = sf::Vector2f(1.0f, 0.0f);
    } else {
        collisionAxis = delta / distance;
    }
    
    // Calculate penetration depth
    const float depth = radiusSum - distance;
    
    // Calculate contact point (on the line between centers)
    const sf::Vector2f contactPoint = centerA + collisionAxis * radiusA;
    
    // Fill manifold
    outManifold.refBody = bodyA;
    outManifold.incBody = bodyB;
    outManifold.collisionAxis = collisionAxis;
    outManifold.depth = depth;
    outManifold.contactPoints.clear();
    outManifold.contactPoints.push_back(contactPoint);
    
    return true;
}

// Returns the edge normals from the vertices.
std::vector<sf::Vector2f> CollisionDetector::GetEdgeNormals(const std::vector<sf::Vector2f>& vertices) {
	std::vector<sf::Vector2f> axes;

	// Find the axis by getting the vertices.
	for (int i = 0; i < vertices.size(); i++) {
		const sf::Vector2f p1 = vertices[i];
		const sf::Vector2f p2 = vertices[(i + 1) % vertices.size()];

		// calculate the vector representing the edge using the two vertices
		const sf::Vector2f edge = p2 - p1;

		// calculate the perpendicular vector (90 degrees anti-clockwise)
		// if we want to do it clockwise we can simply do (edge.y, -edge.x)
		// in our case it does not matter.
		auto edgeNormal = sf::Vector2f(-edge.y, edge.x);

		// normalize the perpendecular
		sf::Vector2f normalNormalized = MathUtils::Normalize(edgeNormal);

		axes.push_back(normalNormalized);
	}

	return axes;
}

// Returns the possible contact point list. No depth check.
std::vector<sf::Vector2f> CollisionDetector::Clip(const std::vector<sf::Vector2f>& incFace, const sf::Vector2f& normal, const float offset) {
	std::vector<sf::Vector2f> contactPoints;

	// we need at 2 points for it to considered as a face.
	if (incFace.size() < 2) return contactPoints;

	// Get the points
	const sf::Vector2f v1 = incFace[0];
	const sf::Vector2f v2 = incFace[1];

	// get the dot product of the normal and points to see and then add the offset to see if the point is inside.
	const float d1 = MathUtils::Dot(normal, v1) - offset;
	const float d2 = MathUtils::Dot(normal, v2) - offset;

	// if dot value is bigger than 0 the point is inside the tunnel we created.
	if (d1 > 0.0f) {
		contactPoints.push_back(v1);
	}

	if (d2 > 0.0f) {
		contactPoints.push_back(v2);
	}

	// check if any of the point is left outside
	if (d1 * d2 < 0.0f) {
		// we'll find out how much outside the point is.
		// Think of this like d1 is +5 and d2 is -5 (meaning d1 is inside 5 meter and d2 is outside by 5 meters).
		// t is lerping factor. so 5 / 10 = 0.5 that means we need to slide that point by half of the distance of the face
		const float t = d1 / (d1 - d2);

		// get the intersection by lerping that much on the v1
		const sf::Vector2f intersectionPoint = v1 + (v2 - v1) * t;

		contactPoints.push_back(intersectionPoint);
	}

	return contactPoints;

}

