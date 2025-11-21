#pragma once
#include <SFML/System/Vector2.hpp>
#include <cmath>

class MathUtils {
public:
    static inline float Dot(const sf::Vector2f& a, const sf::Vector2f& b) {
        return (a.x * b.x) + (a.y * b.y);
    }

    static inline sf::Vector2f Normalize(const sf::Vector2f &vector) {
        // Vector / magnitude
        // magnitude = sqrt (x^2 + y^2)

        float magnitude = std::sqrt((vector.x * vector.x) + (vector.y * vector.y));
        sf::Vector2f n_vector = vector / magnitude;

        return n_vector;
    }
};