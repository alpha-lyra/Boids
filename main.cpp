
#include <random>
#include <iostream>

#include "app.hpp"
#include "boid.hpp"
#include "flock.hpp"



int main() {
  bd::Flock flock1;
  int N{};
  double ds;
  double s{};
  double c{};
  double a{};
  double d{};

  std::cout << "Input the parameters: d, ds, s, a, c "
            << "\n";

  std::cin >> d >> ds >> s >> a >> c;

  std::cout << "Input the number of boids: "
            << "\n";

  std::cin >> N;

  sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
  double screenWidth = desktop.width;
  double screenHeight = desktop.height;

  std::random_device r;
  std::default_random_engine eng(r());
  std::uniform_real_distribution<double> xDist(0, screenWidth);
  std::uniform_real_distribution<double> yDist(0, screenHeight);
  std::uniform_real_distribution<double> vxDist(-1, 1);
  std::uniform_real_distribution<double> vyDist(-1, 1);

  for (int i = 0; i < N; i++) {
    double posX = xDist(eng);
    double posY = yDist(eng);
    double velX = vxDist(eng);
    double velY = vyDist(eng);
    sf::Vector2<double> vel{velX, velY};

    bd::Boid newBoid(posX, posY);
    newBoid.setPar_d(d);
    newBoid.setPar_ds(ds);
    newBoid.setPar_s(s);
    newBoid.setPar_a(a);
    newBoid.setPar_c(c);
    newBoid.setMaxspeed(500);
    newBoid.setVelocity(vel);

    flock1.addBoid(newBoid);
  }
  for (int i = 0; i < N; i++) {
    sf::Vector2<double> posBoid = flock1.getBoid(i).getPosition();
    std::cout << "Posx " << posBoid.x << " Posy " << posBoid.y << "\n";
  }
  for (int i = 0; i < N; i++) {
    sf::Vector2<double> velBoid = flock1.getBoid(i).getVelocity();
    std::cout << "Vx " << velBoid.x << " Vy " << velBoid.y << "\n";
  }

  const double duration = 10.0;  // Total simulation time in seconds
  const double delta_t = 1.;
  double time = 0.;

  while (time < duration) {
    
    flock1.average_distance();
    flock1.average_speed();
    flock1.updateFlock(delta_t);
    time += delta_t;
  }

  return 0;
}