#pragma once
#ifndef APP_HPP
#define APP_HPP

#include "flock.hpp"

#include "SFML/Graphics/RenderWindow.hpp"
#include "SFML/Graphics/CircleShape.hpp"
namespace bd{
class App {
    sf::RenderWindow window;
    double window_width;
    double window_height;

    bd::Flock flock;
    double boidsSize;
    std::vector<sf::CircleShape> shapes;

    void Render();
    void HandleInput();

public:
    App();
    void Run();
};
}

#endif