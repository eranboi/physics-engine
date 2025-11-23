#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include "Physics/PhysicsWorld.h"
#include "Physics/Rigidbody.h"
#include "./Debugging/Gizmos.h"


struct SimulationState {
    std::vector<sf::Vector2f> positions;
    std::vector<sf::Vector2f> velocities;
    std::vector<float> angularVelocities;
};


class Application {
public:
    Application();
    ~Application();
    void Run();

private:
    void ProcessEvents();
    void Update(float dt);
    void Render();
    void InitScene();
    void LoadState(int index);
    void SaveState();

    bool prevSpaceDown = false;
    bool prevRightDown = false;
    bool prevLeftDown = false;
    bool paused = false;
    bool stepForward = false;
    bool stepBackward = false;
    std::vector<SimulationState> history;
    int historyIndex = -1;

    const float PPU = 100.0f;
    const float FIXED_TIME_STEP = 1.0f / 60.0f;

    sf::RenderWindow window;
    PhysicsWorld physicsWorld;
    std::vector<Rigidbody*> bodies;

    sf::CircleShape ballShape;
    sf::ConvexShape polygonShape;

    sf::Font font;
    sf::Text debugText;
    std::string info;

    sf::Clock fpsClock;
    int frameCount;
    float physicsTime;
};

