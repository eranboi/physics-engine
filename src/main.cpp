#include <SFML/Graphics.hpp>
#include <Physics/Rigidbody.h>
#include <Physics/PhysicsWorld.h>
#include <vector>
#include <iostream>
#include <SFML/Graphics/Text.hpp>

int main()
{
    const float PPU = 100.0f; // 1 Meter = 100 Pixels
    sf::Clock clock;

    // Create the main window
    sf::RenderWindow window(sf::VideoMode({ 800, 600 }), "SFML Physics Engine");
    // window.setFramerateLimit(60); // Uncomment to cap FPS

    sf::Font font;
    // Default Windows font. If it fails, ensure a ttf file exists in the project directory.
    if (!font.openFromFile("C:/Windows/Fonts/arial.ttf")) {
        // Output error if file not found (Linux/Mac or missing file)
        std::cout << "Failed to load font! Please add a font file." << std::endl;
    }

    sf::Text debugText(font);
    debugText.setCharacterSize(14); // Pixel size
    debugText.setFillColor(sf::Color::White);
    debugText.setPosition({ 10.f, 10.f });

    // Create a circle shape to represent the rigidbody
    sf::CircleShape shape(0.1f * PPU);
    shape.setFillColor(sf::Color::Green);
    shape.setOrigin(sf::Vector2f(0.1f, 0.1f) * PPU);

    // Create a physics world
    PhysicsWorld physicsWorld;

    // Create a vector to store rigidbodies
    std::vector<Rigidbody*> bodies;

    int columns = 20;     // 20 balls fit side-by-side
    float startX = 1.0f;  // Left start offset
    float startY = 0.5f;  // Top start offset
    float spacing = 0.3f; // Spacing between balls (0.2 diameter + 0.1 gap)

    for (int i = 0; i < 200; i++)
    {
        int col = i % columns;
        int row = i / columns;

        // Restitution: Random between 0.6 and 1.0
        float randomRestitution = 0.6f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 0.4f));

        // Create Body: Mass=1.0, Friction=0.5, Damping=0.995, Radius=0.1, Restitution=Random
        Rigidbody* body = new Rigidbody(1.0f, 0.5f, 0.995f, 0.1f, randomRestitution);

        // Grid Position
        float xPos = startX + (col * spacing);
        float yPos = startY + (row * spacing);
        body->position = sf::Vector2f(xPos, yPos);

        // Random Velocities
        float randVelX = -3.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 6.0f));
        float randVelY = 1.0f + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / 4.0f));
        body->velocity = sf::Vector2f(randVelX, randVelY);

        physicsWorld.AddBody(body);
        bodies.push_back(body);
    }

    // Fixed time step for physics updates
    float fixedDeltaTime = 1.0f / 60.f;
    float accumulator = 0.f;
    float dt = 0;

    sf::Clock fpsClock;
    int frameCount = 0;
    float physicsTimeMicroseconds = 0.f;

    // Start the game loop
    while (window.isOpen())
    {
        // Process events
        while (const std::optional event = window.pollEvent())
        {
            // Close window: exit
            if (event->is<sf::Event::Closed>())
                window.close();
        }

        // Physics time step logic
        dt = clock.restart().asSeconds();
        if (dt > 0.25f) dt = 0.25f; // Cap dt to prevent spiral of death
        accumulator += dt;

        sf::Clock physicsTimer;

        // Update physics world with fixed time step
        while (accumulator >= fixedDeltaTime)
        {
            physicsWorld.Step(fixedDeltaTime);
            accumulator -= fixedDeltaTime;
        }

        // Get physics calculation duration in microseconds
        physicsTimeMicroseconds = physicsTimer.getElapsedTime().asMicroseconds();

        // Clear screen
        window.clear();

        // Render all bodies
        for (Rigidbody* body : bodies)
        {
            // Update the shape's position based on the rigidbody's position
            shape.setPosition(body->position * PPU);
            window.draw(shape);
        }

        // FPS Counter Logic
        frameCount++;
        // Update screen text every 1 second
        if (fpsClock.getElapsedTime().asSeconds() >= 1.0f)
        {
            int fps = frameCount;
            frameCount = 0;
            fpsClock.restart();

            // Create debug string
            std::string info = "FPS: " + std::to_string(fps) + "\n" +
                "Entities: " + std::to_string(bodies.size()) + "\n" +
                "Physics Time: " + std::to_string((int)physicsTimeMicroseconds) + " us";

            debugText.setString(info);
        }

        window.draw(debugText);

        // Update the window
        window.display();
    }

    // Clean up memory
    for (Rigidbody* body : bodies)
    {
        delete body;
    }
    bodies.clear();
}