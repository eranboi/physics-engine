#include <SFML/Graphics.hpp>
#include <Physics/Rigidbody.h>
#include <Physics/PhysicsWorld.h>
#include <vector>

int main()
{
    const float PPU = 100.0f; // 1 Meter = 100 Pixels
    sf::Clock clock;

    // Create the main window
    sf::RenderWindow window(sf::VideoMode({ 800, 600 }), "SFML window");
    window.setFramerateLimit(60);

    // Create a circle shape to represent the rigidbody
    sf::CircleShape shape(0.3f * PPU);
    shape.setFillColor(sf::Color::Green);
    shape.setOrigin(sf::Vector2f(0.3f, 0.3f) * PPU);

    // Create a physics world
    PhysicsWorld physicsWorld;

    // Create a vector to store rigidbodies
    std::vector<Rigidbody*> bodies;

    // Create 10 rigidbodies
    for (int i = 0; i < 10; i++)
    {
        // Create a rigidbody (mass, friction, damping, radius, restitution)
        Rigidbody* body = new Rigidbody(1.0f, 0.5f, 0.99f, 0.3f, 0.99f);

        // Set initial position
        body->position = sf::Vector2f(0.5f + (i * 0.5f), 1.0f + (i % 2 * 0.5f));

        // Set initial velocity in X axis
        body->velocity = sf::Vector2f(2.0f + i * 0.5f, 0.0f);

        // Add the rigidbody to the physics world
        physicsWorld.AddBody(body);
        bodies.push_back(body);
    }

    float fixedDeltaTime = 1.0f / 60.f; // Assuming a fixed time step for physics updates
    float accumulator = 0.f;
    float dt = 0;

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

        // Physics time step
        dt = clock.restart().asSeconds();
        if (dt > 0.25f) dt = 0.25f;
        accumulator += dt;

        // Update physics world with fixed time step
        while (accumulator >= fixedDeltaTime)
        {
            physicsWorld.Step(fixedDeltaTime);
            accumulator -= fixedDeltaTime;
        }

        // Clear screen
        window.clear();

        // Render all bodies
        for (Rigidbody* body : bodies)
        {
            // Update the shape's position based on the rigidbody's position
            shape.setPosition(body->position * PPU);
            window.draw(shape);
        }

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