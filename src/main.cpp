#include <SFML/Graphics.hpp>
#include <Physics/Rigidbody.h>
#include <Physics/PhysicsWorld.h>

int main()
{
    sf::Clock clock;
    // Create the main window
    sf::RenderWindow window(sf::VideoMode({ 800, 600 }), "SFML window");

    // Create a circle shape to represent the rigidbody
	sf::CircleShape shape(30.f);
	shape.setFillColor(sf::Color::Green);
    shape.setOrigin(sf::Vector2f(30.f, 30.f));

    // Create a rigidbody
	Rigidbody body(1.0f, 0.5f, 0.98f);
    body.position = sf::Vector2f(400.f, 100.f);

    // Create a physics world
	PhysicsWorld physicsWorld;

	// Add the rigidbody to the physics world
	physicsWorld.AddBody(&body);


	float fixedDeltaTime = 1 / 60.f; // Assuming a fixed time step for physics updates
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
        accumulator += dt;

		// Update physics world with fixed time step
        while (accumulator >= fixedDeltaTime)
        {
            physicsWorld.Step(fixedDeltaTime);
            accumulator -= fixedDeltaTime;
		}

		// Update the shape's position based on the rigidbody's position
		shape.setPosition(body.position);
        
        window.clear();
		window.draw(shape);
        window.display();
    }
}