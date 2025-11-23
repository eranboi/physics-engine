#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include <cmath>

// Gizmo primitives
struct GizmoLine {
    sf::Vector2f start;
    sf::Vector2f end;
    sf::Color color;
    float thickness;
};

struct GizmoCircle {
    sf::Vector2f center;
    float radius;
    sf::Color color;
    bool filled;
};

struct GizmoRect {
    sf::Vector2f position;
    sf::Vector2f size;
    float rotation;
    sf::Color color;
    bool filled;
};

struct GizmoText {
    sf::Vector2f position;
    std::string text;
    sf::Color color;
    unsigned int size;
};

class Gizmos {
public:
    // Singleton pattern
    static Gizmos& Instance() {
        static Gizmos instance;
        return instance;
    }

    // Initialize
    static void Init(sf::RenderWindow* window, const sf::Font* font, float pixelsPerUnit = 100.0f) {
        auto& inst = Instance();
        inst.window = window;
        inst.font = font;
        inst.PPU = pixelsPerUnit;
    }

    // === DRAWING FUNCTIONS ===

    static void DrawLine(sf::Vector2f start, sf::Vector2f end, sf::Color color = sf::Color::White, float thickness = 1.0f) {
        Instance().lines.push_back({ start, end, color, thickness });
    }

    static void DrawRay(sf::Vector2f origin, sf::Vector2f direction, sf::Color color = sf::Color::White, float thickness = 1.0f) {
        Instance().lines.push_back({ origin, origin + direction, color, thickness });
    }

    static void DrawCircle(sf::Vector2f center, float radius, sf::Color color = sf::Color::White, bool filled = false) {
        Instance().circles.push_back({ center, radius, color, filled });
    }

    static void DrawWireCircle(sf::Vector2f center, float radius, sf::Color color = sf::Color::White) {
        DrawCircle(center, radius, color, false);
    }

    static void DrawRect(sf::Vector2f position, sf::Vector2f size, sf::Color color = sf::Color::White, bool filled = false, float rotation = 0.0f) {
        Instance().rects.push_back({ position, size, rotation, color, filled });
    }

    static void DrawWireRect(sf::Vector2f position, sf::Vector2f size, sf::Color color = sf::Color::White, float rotation = 0.0f) {
        DrawRect(position, size, color, false, rotation);
    }

    static void DrawArrow(sf::Vector2f start, sf::Vector2f end, sf::Color color = sf::Color::Yellow, float thickness = 2.0f, float arrowHeadSize = 0.15f) {
        DrawLine(start, end, color, thickness);

        sf::Vector2f direction = end - start;
        float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);

        if (length < 0.001f) return;

        direction /= length; // Normalize
        sf::Vector2f perpendicular(-direction.y, direction.x);

        float headLength = arrowHeadSize;
        float headWidth = arrowHeadSize * 0.5f;

        sf::Vector2f arrowBase = end - direction * headLength;
        sf::Vector2f arrowLeft = arrowBase + perpendicular * headWidth;
        sf::Vector2f arrowRight = arrowBase - perpendicular * headWidth;

        DrawLine(end, arrowLeft, color, thickness);
        DrawLine(end, arrowRight, color, thickness);
        DrawLine(arrowLeft, arrowRight, color, thickness);
    }

    static void DrawCross(sf::Vector2f center, float size = 0.1f, sf::Color color = sf::Color::Red, float thickness = 2.0f) {
        float half = size * 0.5f;
        DrawLine(center + sf::Vector2f(-half, -half), center + sf::Vector2f(half, half), color, thickness);
        DrawLine(center + sf::Vector2f(-half, half), center + sf::Vector2f(half, -half), color, thickness);
    }

    static void DrawPoint(sf::Vector2f position, sf::Color color = sf::Color::Red, float size = 0.05f) {
        DrawCircle(position, size, color, true);
    }

    static void DrawText(sf::Vector2f position, const std::string& text, sf::Color color = sf::Color::White, unsigned int size = 12) {
        Instance().texts.push_back({ position, text, color, size });
    }

    static void DrawPolygon(const std::vector<sf::Vector2f>& vertices, sf::Color color = sf::Color::White, bool filled = false, float thickness = 1.0f) {
        if (vertices.size() < 2) return;

        for (size_t i = 0; i < vertices.size(); i++) {
            sf::Vector2f start = vertices[i];
            sf::Vector2f end = vertices[(i + 1) % vertices.size()];
            DrawLine(start, end, color, thickness);
        }
    }

    static void DrawWireCube(sf::Vector2f center, sf::Vector2f size, sf::Color color = sf::Color::White, float rotation = 0.0f) {
        DrawWireRect(center - size * 0.5f, size, color, rotation);
    }

    // === UTILITY FUNCTIONS ===

    static void Clear() {
        auto& inst = Instance();
        inst.lines.clear();
        inst.circles.clear();
        inst.rects.clear();
        inst.texts.clear();
    }

    static void Render() {
        auto& inst = Instance();
        if (!inst.window) return;

        inst.RenderLines();
        inst.RenderCircles();
        inst.RenderRects();
        inst.RenderTexts();
    }

    static void SetPixelsPerUnit(float ppu) {
        Instance().PPU = ppu;
    }

private:
    Gizmos() = default;
    Gizmos(const Gizmos&) = delete;
    Gizmos& operator=(const Gizmos&) = delete;

    // === INTERNAL RENDERING ===

    void RenderLines() const {
        for (const auto& line : lines) {
            sf::Vector2f start = line.start * PPU;
            sf::Vector2f end = line.end * PPU;

            if (line.thickness <= 1.0f) {
                sf::Vertex v1;
                v1.position = start;
                v1.color = line.color;

                sf::Vertex v2;
                v2.position = end;
                v2.color = line.color;

                sf::Vertex vertices[2] = { v1, v2 };

                window->draw(vertices, 2, sf::PrimitiveType::Lines);
            }
            else {
                sf::Vector2f direction = end - start;
                float length = std::sqrt(direction.x * direction.x + direction.y * direction.y);
                float angle = std::atan2(direction.y, direction.x) * 180.0f / 3.14159f;

                sf::RectangleShape rect(sf::Vector2f(length, line.thickness));
                rect.setPosition(start);
                rect.setRotation(sf::degrees(angle));
                rect.setFillColor(line.color);
                window->draw(rect);
            }
        }
    }

    void RenderCircles() const {
        for (const auto& circle : circles) {
            float radiusPx = circle.radius * PPU;
            sf::CircleShape shape(radiusPx);
            shape.setPosition(circle.center * PPU);
            shape.setOrigin(sf::Vector2f(radiusPx, radiusPx));

            if (circle.filled) {
                shape.setFillColor(circle.color);
            }
            else {
                shape.setFillColor(sf::Color::Transparent);
                shape.setOutlineColor(circle.color);
                shape.setOutlineThickness(1.0f);
            }

            window->draw(shape);
        }
    }

    void RenderRects() const {
        for (const auto& rect : rects) {
            sf::RectangleShape shape(rect.size * PPU);
            shape.setPosition(rect.position * PPU);
            shape.setRotation(sf::degrees(rect.rotation * 180.0f / 3.14159f));

            if (rect.filled) {
                shape.setFillColor(rect.color);
            }
            else {
                shape.setFillColor(sf::Color::Transparent);
                shape.setOutlineColor(rect.color);
                shape.setOutlineThickness(1.0f);
            }

            window->draw(shape);
        }
    }

    void RenderTexts() const {
        if (!font) return;

        for (const auto& textData : texts) {
            sf::Text text(*font);
            text.setString(textData.text);
            text.setCharacterSize(textData.size);
            text.setFillColor(textData.color);
            text.setPosition(textData.position * PPU);
            window->draw(text);
        }
    }

    // Members
    sf::RenderWindow* window = nullptr;
    const sf::Font* font = nullptr;
    float PPU = 100.0f;

    std::vector<GizmoLine> lines;
    std::vector<GizmoCircle> circles;
    std::vector<GizmoRect> rects;
    std::vector<GizmoText> texts;
};