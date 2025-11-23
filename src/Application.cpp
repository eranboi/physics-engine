#include "Application.h"
#include <iostream>
#include <sstream>

Application::Application()
    : window(sf::VideoMode({ 1600, 900 }), "Physics Engine"),
    physicsWorld(16, 9),
    debugText(font),
    frameCount(0),
    physicsTime(0.0f)
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
    for (const Rigidbody* body : bodies) {
        delete body;
    }
    bodies.clear();
}

void Application::InitScene() {
    constexpr int bodyCount = 50;

    Gizmos::Init(&window, &font, PPU);

	// Changes with window size
    constexpr float worldWidth = 16.0f;
    constexpr float worldHeight = 9.0f;

    constexpr float wallThickness = 1.0f;
    constexpr float halfThick = wallThickness / 2.0f;

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


    constexpr float shapeSize = 0.3f;
    constexpr float spacing = 0.25f;

    const int columns = static_cast<int>(sqrt(bodyCount * 1.77f));

    const float totalGroupWidth = columns * spacing;
    const float rows = std::ceil(static_cast<float>(bodyCount) / columns);
    const float totalGroupHeight = rows * spacing;

    const float startX = (worldWidth - totalGroupWidth) / 2.0f;
    const float startY = (worldHeight - totalGroupHeight) / 2.0f;

    for (int i = 0; i < bodyCount; i++) {
        const int col = i % columns;
        const int row = i / columns;

        const float jitter = (static_cast<float>(rand()) / RAND_MAX) * 0.02f;
        const float xPos = startX + (col * spacing) + jitter;
        const float yPos = startY + (row * spacing);

        const float randomRestitution = 0.2f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 0.7f));

        Rigidbody* body = nullptr;
        
        // Random object creation
        if (rand() % 2 == 0) {
            body = Rigidbody::CreateBox(shapeSize, shapeSize, 1.0f, randomRestitution);
        }
        else {
            // Get local points for the triangle
            // Create triangle finds the center and fixes the positional issues of the vertices
            // After that we'll move the bodies to their appropriate positions.

            constexpr float s = shapeSize;

            constexpr sf::Vector2f p1(0.0f, -s / 2);
            constexpr sf::Vector2f p2(s, s);
            constexpr sf::Vector2f p3(-s, s);

            body = Rigidbody::CreateTriangle(p1, p2, p3, 1.0f, randomRestitution);
        }

        // Move to body to the position and give it initial values.
        if (body != nullptr) {
            body->position = sf::Vector2f(xPos, yPos);

            // Random velocity
            const float randVelX = -4.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 8.0f));
            const float randVelY = -4.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 8.0f));
            body->velocity = sf::Vector2f(randVelX, randVelY);

            body->color = sf::Color(66, 15, 47);
          
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

    const bool spaceDown = sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::Space);
    if (spaceDown && !prevSpaceDown) {
        paused = !paused;
    }
    prevSpaceDown = spaceDown;

    const bool rightDown = sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::Right);
    if (rightDown && !prevRightDown) {
        stepForward = true;
    }
    prevRightDown = rightDown;

    const bool leftDown = sf::Keyboard::isKeyPressed(sf::Keyboard::Scan::Left);
    if (leftDown && !prevLeftDown) {
        stepBackward = true;
    }
    prevLeftDown = leftDown;
}

void Application::Update(float dt) {
    if (paused) {
        // Step forward
        if (stepForward) {
            Gizmos::Clear();
            physicsWorld.Step(dt);
            SaveState();
            stepForward = false;
        }
        // Step back
        else if (stepBackward) {
            Gizmos::Clear();
            if (historyIndex > 0) {
                historyIndex--;
                LoadState(historyIndex);
            }
            stepBackward = false;
        }

        // If paused return
        return;
    }
    Gizmos::Clear();
    physicsWorld.Step(dt);
    SaveState();
}

void Application::Render() {
    window.clear();
    // Render all bodies using the shared shape (Idk if that's helping with the performance or not)
    for (Rigidbody* body : bodies) {
        body->OnDrawGizmos();
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
    
    // Gizmos::Render();

    // Calculate and display FPS and Physics stats
    frameCount++;
    if (fpsClock.getElapsedTime().asSeconds() >= 1.0f) {
        std::ostringstream ss;

        ss << "Debug Info\n"
            << "FPS:           " << frameCount << "\n"
            << "Entities:      " << bodies.size() << "\n"
            << "Physics Time:  " << physicsTime << " s\n"
            << "Game State:    " << (paused ? "Paused" : "Running") << "\n"
            << "History Size:  " << history.size() << "\n"
            << "Current Step:  " << (historyIndex + 1) << "\n";

        info = ss.str();


        debugText.setString(info);
        frameCount = 0;
        fpsClock.restart();
    }

    window.draw(debugText);
    window.display();
}

void Application::LoadState(int index) const {
    if (index < 0 || index >= history.size()) return;

    const auto& s = history[index];
    for (int i = 0; i < bodies.size(); i++) {
        bodies[i]->position = s.positions[i];
        bodies[i]->velocity = s.velocities[i];
        bodies[i]->angularVelocity = s.angularVelocities[i];
    }
}

void Application::SaveState()
{
    SimulationState s;
    s.positions.reserve(bodies.size());
    s.velocities.reserve(bodies.size());
    s.angularVelocities.reserve(bodies.size());

    for (const auto* b : bodies) {
        s.positions.push_back(b->position);
        s.velocities.push_back(b->velocity);
        s.angularVelocities.push_back(b->angularVelocity);
    }

    // Step back
    if (historyIndex < static_cast<int>(history.size()) - 1) {
        history.erase(history.begin() + historyIndex + 1, history.end());
    }

    history.push_back(s);
    historyIndex = history.size() - 1;
}

