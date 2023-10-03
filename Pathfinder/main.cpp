
#include <iostream>
#include "Interface.h"

int main() {

    sf::VideoMode display = sf::VideoMode::getDesktopMode();
    sf::RenderWindow Window(sf::VideoMode(display.width * 0.7f, display.height * 0.7f), "Pathfinder");
    Window.setFramerateLimit(60);

    Interface* interface = new Interface();

    while (Window.isOpen())
    {
        sf::Event event;
        while (Window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                Window.close();
        }

        Window.clear();

        interface->Update(Window);

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::T)) interface->Test(Window);

        Window.display();
    }

    delete interface;

	return 0;
}





