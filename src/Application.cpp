#include "Application.h"
#include <iostream>
#include <sstream>


Application::Application()
    : window(sf::VideoMode({ 1600, 900 }), "Physics Engine"),
    physicsWorld(16, 9, new ImpulseSolver()),
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

void Application::CreateStaticWalls(float worldWidth, float worldHeight, float wallThickness) {
    float halfThick = wallThickness / 2.0f;

    // Config for static walls (mass = 0 means infinite mass, thus static)
    RigidbodyConfig wallConfig;
    wallConfig.mass = 0.0f;
    wallConfig.restitution = 0.3f;
    wallConfig.friction = 0.6f;

    constexpr sf::Color wallColor(40, 40, 40);

    // Floor
    Rigidbody* floor = Rigidbody::CreateBox(worldWidth, wallThickness, wallConfig);
    floor->position = sf::Vector2f(worldWidth / 2.0f, worldHeight - halfThick);
    floor->color = wallColor;
    physicsWorld.AddBody(floor);
    bodies.push_back(floor);

    // Ceiling
    Rigidbody* ceiling = Rigidbody::CreateBox(worldWidth, wallThickness, wallConfig);
    ceiling->position = sf::Vector2f(worldWidth / 2.0f, halfThick);
    ceiling->color = wallColor;
    physicsWorld.AddBody(ceiling);
    bodies.push_back(ceiling);

    // Left Wall
    Rigidbody* leftWall = Rigidbody::CreateBox(wallThickness, worldHeight - (wallThickness * 2), wallConfig);
    leftWall->position = sf::Vector2f(halfThick, worldHeight / 2.0f);
    leftWall->color = wallColor;
    physicsWorld.AddBody(leftWall);
    bodies.push_back(leftWall);

    // Right Wall
    Rigidbody* rightWall = Rigidbody::CreateBox(wallThickness, worldHeight - (wallThickness * 2), wallConfig);
    rightWall->position = sf::Vector2f(worldWidth - halfThick, worldHeight / 2.0f);
    rightWall->color = wallColor;
    physicsWorld.AddBody(rightWall);
    bodies.push_back(rightWall);
}

void Application::CreatePlatforms(float worldWidth, float worldHeight) {
    constexpr sf::Color platformColor(80, 80, 90);
    constexpr sf::Color slopeColor(90, 70, 70);

    // Platform configurations
    RigidbodyConfig lowFrictionConfig;
    lowFrictionConfig.mass = 0.0f;
    lowFrictionConfig.restitution = 0.2f;
    lowFrictionConfig.friction = 0.1f;

    RigidbodyConfig normalFrictionConfig;
    normalFrictionConfig.mass = 0.0f;
    normalFrictionConfig.restitution = 0.3f;
    normalFrictionConfig.friction = 0.6f;

    RigidbodyConfig highFrictionConfig;
    highFrictionConfig.mass = 0.0f;
    highFrictionConfig.restitution = 0.2f;
    highFrictionConfig.friction = 1.2f;

    // Left side - Slippery slope
    {
        std::vector<sf::Vector2f> slopeVertices;
        constexpr float slopeWidth = 3.0f;
        constexpr float slopeHeight = 1.5f;

        // Create a slope
        slopeVertices.push_back(sf::Vector2f(-slopeWidth / 2.0f, slopeHeight / 2.0f));  // Bottom left
        slopeVertices.push_back(sf::Vector2f(slopeWidth / 2.0f, slopeHeight / 2.0f));   // Bottom right
        slopeVertices.push_back(sf::Vector2f(slopeWidth / 2.0f, -slopeHeight / 2.0f));  // Top right

        Rigidbody* slope = Rigidbody::CreatePolygon(slopeVertices, lowFrictionConfig);
        slope->position = sf::Vector2f(3.5f, worldHeight - 3.0f);
        slope->color = sf::Color(100, 150, 200);
        physicsWorld.AddBody(slope);
        bodies.push_back(slope);
    }

    // Center - Normal platform
    {
        Rigidbody* platform = Rigidbody::CreateBox(4.0f, 0.3f, normalFrictionConfig);
        platform->position = sf::Vector2f(worldWidth / 2.0f, worldHeight - 4.5f);
        platform->color = platformColor;
        physicsWorld.AddBody(platform);
        bodies.push_back(platform);
    }

    // Right side - Sticky slope
    {
        std::vector<sf::Vector2f> slopeVertices;
        constexpr float slopeWidth = 3.0f;
        constexpr float slopeHeight = 1.5f;

        slopeVertices.push_back(sf::Vector2f(-slopeWidth / 2.0f, -slopeHeight / 2.0f));  // Top left
        slopeVertices.push_back(sf::Vector2f(-slopeWidth / 2.0f, slopeHeight / 2.0f));   // Bottom left
        slopeVertices.push_back(sf::Vector2f(slopeWidth / 2.0f, slopeHeight / 2.0f));    // Bottom right

        Rigidbody* slope = Rigidbody::CreatePolygon(slopeVertices, highFrictionConfig);
        slope->position = sf::Vector2f(worldWidth - 3.5f, worldHeight - 3.0f);
        slope->color = sf::Color(200, 100, 100);
        physicsWorld.AddBody(slope);
        bodies.push_back(slope);
    }

    // Small platform steps
    {
        // Low friction step
        Rigidbody* step1 = Rigidbody::CreateBox(1.5f, 0.2f, lowFrictionConfig);
        step1->position = sf::Vector2f(3.0f, worldHeight - 2.0f);
        step1->color = sf::Color(100, 150, 200);
        physicsWorld.AddBody(step1);
        bodies.push_back(step1);

        // High friction step
        Rigidbody* step2 = Rigidbody::CreateBox(1.5f, 0.2f, highFrictionConfig);
        step2->position = sf::Vector2f(worldWidth - 3.0f, worldHeight - 2.0f);
        step2->color = sf::Color(200, 100, 100);
        physicsWorld.AddBody(step2);
        bodies.push_back(step2);
    }

    // Rotated platform
    {
        Rigidbody* rotatedPlatform = Rigidbody::CreateBox(2.5f, 0.25f, normalFrictionConfig);
        rotatedPlatform->position = sf::Vector2f(worldWidth / 2.0f, worldHeight - 6.5f);
        rotatedPlatform->rotation = 0.3f;
        rotatedPlatform->color = platformColor;
        physicsWorld.AddBody(rotatedPlatform);
        bodies.push_back(rotatedPlatform);
    }
}

void Application::CreateDynamicBodies(float worldWidth, float worldHeight) {
    // Shape colors
    constexpr sf::Color boxColor(255, 107, 107);
    constexpr sf::Color triangleColor(78, 205, 196);
    constexpr sf::Color circleColor(255, 195, 0);
    constexpr sf::Color hexagonColor(199, 125, 255);
    constexpr sf::Color pentagonColor(255, 159, 243);

    // Configuration for dynamic bodies
    RigidbodyConfig dynamicConfig;
    dynamicConfig.mass = 10000000.f;
    dynamicConfig.friction = 0.4f;
    dynamicConfig.damping = 0.1f;
    dynamicConfig.angularDamping = .5f;

    // Grid parameters for initial placement
    constexpr float spacing = .75f;
    constexpr int columns = 16;
    constexpr int rows = 46;

    const float startX = (worldWidth - (columns * spacing)) / 2.0f + spacing / 2.0f;
    constexpr float startY = 2.0f;

    int shapeIndex = 0;

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < columns; col++) {
            const float xPos = startX + (col * spacing);
            const float yPos = startY + (row * spacing);

            // Add small random jitter to positions
            const float jitterX = ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 0.1f;
            const float jitterY = ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 0.1f;

            // Random restitution
            dynamicConfig.restitution = 0.3f + static_cast<float>(rand()) / (RAND_MAX / 0.5f);

            Rigidbody* body = nullptr;

            // Cycle through different shapes
            int shapeType = shapeIndex % 5;

            switch (shapeType) {
                case 0: { // Box
                    constexpr float size = 0.35f;
                    body = Rigidbody::CreateBox(size, size, dynamicConfig);
                    body->color = boxColor;
                    break;
                }

                case 1: { // Triangle
                    constexpr float size = 0.4f;
                    constexpr sf::Vector2f p1(0.0f, -size * 0.7f);
                    constexpr sf::Vector2f p2(size * 0.8f, size * 0.5f);
                    constexpr sf::Vector2f p3(-size * 0.8f, size * 0.5f);
                    body = Rigidbody::CreateTriangle(p1, p2, p3, dynamicConfig);
                    body->color = triangleColor;
                    break;
                }

                case 2: { // Circle
                    constexpr float radius = 0.2f;
                    body = Rigidbody::CreateCircle(radius, dynamicConfig);
                    body->color = circleColor;
                    break;
                }

                case 3: { // Hexagon
                    std::vector<sf::Vector2f> hexVertices;
                    constexpr float radius = 0.25f;
                    constexpr int sides = 6;
                    constexpr float angleStep = 2.0f * 3.14159f / sides;

                    for (int i = 0; i < sides; i++) {
                        float angle = -i * angleStep;  // CCW in SFML
                        hexVertices.push_back(sf::Vector2f(
                            radius * std::cos(angle),
                            radius * std::sin(angle)
                        ));
                    }

                    body = Rigidbody::CreatePolygon(hexVertices, dynamicConfig);
                    body->color = hexagonColor;
                    break;
                }

                case 4: { // Pentagon
                    std::vector<sf::Vector2f> pentVertices;
                    constexpr float radius = 0.25f;
                    constexpr int sides = 5;
                    constexpr float angleStep = 2.0f * 3.14159f / sides;
                    constexpr float angleOffset = -3.14159f / 2.0f; // Start from top

                    for (int i = 0; i < sides; i++) {
                        float angle = -i * angleStep + angleOffset;  // CCW in SFML
                        pentVertices.push_back(sf::Vector2f(
                            radius * std::cos(angle),
                            radius * std::sin(angle)
                        ));
                    }

                    body = Rigidbody::CreatePolygon(pentVertices, dynamicConfig);
                    body->color = pentagonColor;
                    break;
                }
            }

            if (body != nullptr) {
                body->position = sf::Vector2f(xPos + jitterX, yPos + jitterY);

                // Add some random initial velocity for chaos
                constexpr float velMagnitude = 2.0f;
                const float randVelX = ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * velMagnitude;
                const float randVelY = ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * velMagnitude;
                body->velocity = sf::Vector2f(randVelX, randVelY);

                // Random initial rotation
                const float randAngVel = ((static_cast<float>(rand()) / RAND_MAX) - 0.5f) * 3.0f;
                body->angularVelocity = randAngVel;

                physicsWorld.AddBody(body);
                bodies.push_back(body);
            }

            shapeIndex++;
        }
    }
}

void Application::InitScene() {
    Gizmos::Init(&window, &font, PPU);

    // World dimensions
    constexpr float worldWidth = 16.0f;
    constexpr float worldHeight = 9.0f;
    constexpr float wallThickness = 1.0f;

    // Create scene in stages
    CreateStaticWalls(worldWidth, worldHeight, wallThickness);
    CreatePlatforms(worldWidth, worldHeight);
    CreateDynamicBodies(worldWidth, worldHeight);
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

    Gizmos::Render();

    // Calculate and display FPS and Physics stats
    frameCount++;
    if (fpsClock.getElapsedTime().asSeconds() >= 1.0f) {
        std::ostringstream ss;

        ss << "Debug Info\n"
            << "FPS:           " << frameCount << "\n"
            << "Entities:      " << bodies.size() << "\n"
            << "Manifolds:     " << physicsWorld.GetManifoldCount() << "\n"
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