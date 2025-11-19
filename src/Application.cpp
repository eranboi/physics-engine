#include "Application.h"
#include <iostream>
#include <string>

Application::Application()
    : window(sf::VideoMode({ 1600, 900 }), "Physics Engine"),
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

    // Configure the shared base shape for rendering
    baseShape.setRadius(0.1f * PPU);
    baseShape.setOrigin(sf::Vector2f(0.1f * PPU, 0.1f * PPU));
    baseShape.setFillColor(sf::Color::Green);

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
    int ballCount = 1250;

	// Changes with window size
    float worldWidth = 16.0f;
    float worldHeight = 9.0f;

    float ballRadius = 0.1f;
    float spacing = .25f;


    int columns = static_cast<int>(sqrt(ballCount * 1.77f));

    float totalGroupWidth = columns * spacing;
    float rows = std::ceil((float)ballCount / columns);
    float totalGroupHeight = rows * spacing;

    float startX = (worldWidth - totalGroupWidth) / 2.0f;
    float startY = (worldHeight - totalGroupHeight) / 2.0f;

    for (int i = 0; i < ballCount; i++) {
        int col = i % columns;
        int row = i / columns;

		// Random restitution between 0.9 and 1.0
        float randomRestitution = 0.9f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 0.1f));

		// Create Rigidbody
        Rigidbody* body = new Rigidbody(1.0f, 0.5f, 0.999f, ballRadius, randomRestitution);

		// Add slight random jitter to positions to avoid perfect grid alignment
        float jitter = (static_cast<float>(rand()) / RAND_MAX) * 0.02f;

        float xPos = startX + (col * spacing) + jitter;
        float yPos = startY + (row * spacing);

        body->position = sf::Vector2f(xPos, yPos);

		// Random initial velocity
        float randVelX = -2.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 4.0f));
        float randVelY = -2.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 4.0f));
        body->velocity = sf::Vector2f(randVelX, randVelY);

		// Random color based on index
        body->color = sf::Color(255, 100 + (i % 155), 100); 

        physicsWorld.AddBody(body);
        bodies.push_back(body);
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

    // Render all bodies using the shared shape
    for (Rigidbody* body : bodies) {
        baseShape.setPosition(body->position * PPU);
		baseShape.setFillColor(body->color);
        window.draw(baseShape);
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