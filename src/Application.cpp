#include "Application.h"
#include <iostream>
#include <string>
#include <cstdint>

Application::Application()
    : window(sf::VideoMode({ 1600, 900 }), "Physics Engine"),
    physicsWorld(16, 9),
    frameCount(0),
    physicsTime(0.0f),
    debugText(font)
{
    // Load font from file
    if (!font.openFromFile("assets/fonts/poppins.regular.ttf")) {
        std::cout << "Failed to load font!" << std::endl;
    }

    // Configure debug text properties
    debugText.setCharacterSize(14);
    debugText.setFillColor(sf::Color::White);
    debugText.setPosition({ 10.f, 10.f });

    // Initialize the game scene
    InitScene();
}

Application::~Application() {
    // Clean up allocated memory for rigidbodies
    for (Rigidbody* body : bodies) {
        delete body;
    }
    bodies.clear();
}

void Application::InitScene() {
    int bodyCount = 50;

	// Changes with window size
    float worldWidth = 16.0f;
    float worldHeight = 9.0f;

    float wallThickness = 1.0f;
    float halfThick = wallThickness / 2.0f;

    // Floor 
    Rigidbody* floor = Rigidbody::CreateBox(worldWidth, wallThickness, 0.0f, 0.5f);
    floor->position = sf::Vector2f(worldWidth / 2.0f, worldHeight - halfThick);
    floor->color = sf::Color(50, 50, 50);
    physicsWorld.AddBody(floor);
    bodies.push_back(floor);

    // Ceiling
    Rigidbody* ceiling = Rigidbody::CreateBox(worldWidth, wallThickness, 0.0f, 0.5f);
    ceiling->position = sf::Vector2f(worldWidth / 2.0f, halfThick);
    ceiling->color = sf::Color(50, 50, 50);
    physicsWorld.AddBody(ceiling);
    bodies.push_back(ceiling);

    // Left Wall
    Rigidbody* leftWall = Rigidbody::CreateBox(wallThickness, worldHeight - (wallThickness * 2), 0.0f, 0.5f);
    leftWall->position = sf::Vector2f(halfThick, worldHeight / 2.0f);
    leftWall->color = sf::Color(50, 50, 50);
    physicsWorld.AddBody(leftWall);
    bodies.push_back(leftWall);

    // Right Wall
    Rigidbody* rightWall = Rigidbody::CreateBox(wallThickness, worldHeight - (wallThickness * 2), 0.0f, 0.5f);
    rightWall->position = sf::Vector2f(worldWidth - halfThick, worldHeight / 2.0f);
    rightWall->color = sf::Color(50, 50, 50);
    physicsWorld.AddBody(rightWall);
    bodies.push_back(rightWall);


    float shapeSize = 0.3f;
    float spacing = .25f;

    int columns = static_cast<int>(sqrt(bodyCount * 1.77f));

    float totalGroupWidth = columns * spacing;
    float rows = std::ceil((float)bodyCount / columns);
    float totalGroupHeight = rows * spacing;

    float startX = (worldWidth - totalGroupWidth) / 2.0f;
    float startY = (worldHeight - totalGroupHeight) / 2.0f;

    for (int i = 0; i < bodyCount; i++) {
        int col = i % columns;
        int row = i / columns;

        float jitter = (static_cast<float>(rand()) / RAND_MAX) * 0.02f;
        float xPos = startX + (col * spacing) + jitter;
        float yPos = startY + (row * spacing);

        float randomRestitution = 0.2f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 0.7f));

        Rigidbody* body = nullptr;

        // Random object creation
        if (rand() % 2 == 0) {
            body = Rigidbody::CreateBox(shapeSize, shapeSize, 1.0f, randomRestitution);
        }
        else {
            // Get local points for the triangle
            // Create triangle finds the center and fixes the positional issues of the vertices
            // After that we'll move the bodies to their appropriate positions.

            float s = shapeSize;
            
            sf::Vector2f p1(0.0f, -s / 2);
            sf::Vector2f p2(s, s);
            sf::Vector2f p3(-s, s);

            body = Rigidbody::CreateTriangle(p1, p2, p3, 1.0f, randomRestitution);
        }

        // Move to body to the position and give it initial values.
        if (body != nullptr) {
            body->position = sf::Vector2f(xPos, yPos);

            // Random velocity
            float randVelX = -4.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 8.0f));
            float randVelY = -4.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 8.0f));
            body->velocity = sf::Vector2f(randVelX, randVelY);

            body->color = sf::Color(224, 187, 228);
          
            // Add to the simulation
            physicsWorld.AddBody(body);
            bodies.push_back(body);
        }
    }
}

void Application::Run() {
    sf::Clock clock;
    float accumulator = 0.f;

    while (window.isOpen()) {
        ProcessEvents();

        // Calculate delta time
        float dt = clock.restart().asSeconds();

        // Clamp delta time to avoid spiral of death on lag spikes
        if (dt > 0.25f) dt = 0.25f;
        accumulator += dt;

        // Measure physics execution time
        sf::Clock physTimer;

        int steps = 0;
        // Fixed time step update loop
        while (accumulator >= FIXED_TIME_STEP && steps < 2) {
            Update(FIXED_TIME_STEP);
            accumulator -= FIXED_TIME_STEP;
            steps++;
        }

        physicsTime = physTimer.getElapsedTime().asSeconds();

        Render();
    }
}

void Application::ProcessEvents() {
    while (const std::optional event = window.pollEvent()) {
        if (event->is<sf::Event::Closed>())
            window.close();
    }
}

void Application::Update(float dt) {
    physicsWorld.Step(dt);
}

void Application::Render() {
    window.clear();

    // Render all bodies using the shared shape (Idk if that's helping with the performance or not)
    for (Rigidbody* body : bodies) {
        if(body-> shapeType == ShapeType::Circle)
        {
            ballShape.setRadius(body->radius * PPU);
            ballShape.setOrigin(sf::Vector2f(body->radius * PPU, body->radius * PPU));
            ballShape.setPosition(body->position * PPU);
            ballShape.setFillColor(body->color);
            window.draw(ballShape);
        }
        else {

            std::vector<sf::Vector2f> vertices = body->GetTransformedVertices();

            polygonShape.setPointCount(vertices.size());
            for (size_t i = 0; i < vertices.size(); i++) {
                polygonShape.setPoint(i, vertices[i] * PPU);
            }

            polygonShape.setFillColor(body->color);
            window.draw(polygonShape);
        }
    }

    // Calculate and display FPS and Physics stats
    frameCount++;
    if (fpsClock.getElapsedTime().asSeconds() >= 1.0f) {
        info = "FPS: " + std::to_string(frameCount) + "\n" +
            "Entities: " + std::to_string(bodies.size()) + "\n" +
            "Physics: " + std::to_string(physicsTime) + " s";

        debugText.setString(info);
        frameCount = 0;
        fpsClock.restart();
    }

    window.draw(debugText);
    window.display();
}