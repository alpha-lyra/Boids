#include "boid.hpp"

#include <cmath>
#include <iostream>
#include <numeric>

namespace bd {

double distance(const sf::Vector2<double>& vec1,
                const sf::Vector2<double>& vec2) {
  double dX = vec2.x - vec1.x;
  double dY = vec2.y - vec1.y;
  return std::sqrt(dX * dX + dY * dY);
}

double magnitude(const sf::Vector2<double>& vec) {
  return std::sqrt(vec.x * vec.x + vec.y * vec.y);
}

Boid::Boid() : position(0, 0) {}
Boid::Boid(double pos_x, double pos_y) : position(pos_x, pos_y) {}

sf::Vector2<double> Boid::getPosition() const { return position; }
void Boid::setPosition(const sf::Vector2<double>& newPos) { position = newPos; }

sf::Vector2<double> Boid::getVelocity() const { return velocity; }
void Boid::setVelocity(const sf::Vector2<double>& newVel) { velocity = newVel; }

Parameters Boid::getPar() const { return par; }
void Boid::setPar(const Parameters& newPar) { par = newPar; }

void Boid::setPar_d(const double new_d) { par.d = new_d; }
void Boid::setPar_ds(const double new_ds) { par.ds = new_ds; }
void Boid::setPar_s(const double new_s) { par.s = new_s; }
void Boid::setPar_a(const double new_a) { par.a = new_a; }
void Boid::setPar_c(const double new_c) { par.c = new_c; }

double Boid::getMaxspeed() const { return maxspeed; }
void Boid::setMaxspeed(double new_Maxspeed) { maxspeed = new_Maxspeed; } 

sf::Vector2<double> Boid::separation(const std::vector<Boid>& boids) {
  double d = (*this).par.d;
  double ds = (*this).par.ds;
  double s = (*this).par.s;
  sf::Vector2<double> Displacements(0, 0);

  const sf::Vector2<double>& myPosition = (*this).position; 

  for (auto const& boid : boids) {
    const sf::Vector2<double>& otherPosition = boid.position;
    double distance1 = distance(myPosition, otherPosition);
    if (distance1 < d) {
      sf::Vector2<double> displacement = otherPosition - myPosition;
      if (distance1 < ds) {
        Displacements = Displacements + displacement;
      }
    }
  }
  sf::Vector2<double> v1 = -s * Displacements;
  return v1;
}

sf::Vector2<double> Boid::alignment(const std::vector<Boid>& boids) {
  sf::Vector2<double> Velocities(0, 0);
  double a = (*this).par.a;
  double d = (*this).par.d;

  const sf::Vector2<double>& myVelocity = (*this).velocity;

  int N = boids.size();

  for (auto const& boid : boids) {
    double distance1 = distance((*this).position, boid.position);
    if (distance1 < d) {
      sf::Vector2<double> speed = boid.velocity - myVelocity;
      Velocities = Velocities + speed;
    }
  }

  sf::Vector2<double> v2 = a * (1.0 / (N - 1)) * Velocities;
  return v2;
}

sf::Vector2<double> Boid::cohesion(const std::vector<Boid>& boids) {
  sf::Vector2<double> sum_pos(0, 0);

  double c = (*this).par.c;
  double d = (*this).par.d;
  int N = boids.size();

  const sf::Vector2<double>& myPosition = (*this).position;

  for (auto const& boid : boids) {
    double distance1 = distance((*this).position, boid.position);
    if (distance1 < d) {
      sf::Vector2<double> otherPosition = boid.position;

      sum_pos = sum_pos + otherPosition;
    }
  }
  sum_pos = sum_pos - myPosition;
  sf::Vector2<double> xc = (1.0 / (N - 1)) * sum_pos;
  sf::Vector2<double> v3 = c * (xc - myPosition);
  return v3;
}

void Boid::updateVelocity(const std::vector<Boid>& boids) {
  sf::Vector2<double> v1 = separation(boids);
  sf::Vector2<double> v2 = alignment(boids);
  sf::Vector2<double> v3 = cohesion(boids);

  velocity = velocity + v1 + v2 + v3;

  double mag_v = magnitude(velocity);

  if (mag_v > maxspeed) {
    velocity.x /= (mag_v / maxspeed);  
    velocity.y /= (mag_v / maxspeed);
  };
}
void Boid::updatePosition(double const delta_t) {
  position = position + velocity * delta_t;
}

void Boid::borders() {
  sf::VideoMode desktop = sf::VideoMode::getDesktopMode();
  double screenWidth = desktop.width;
  double screenHeight = desktop.height;

  if (position.x < 0.) {
    position.x = screenWidth;
  } else if (position.x > screenWidth) {
    position.x = 0.;
  }
  if (position.y < 0.) {
    position.y =
        screenHeight;  // lo schermo va al contrario quindi bordo superiore
  } else if (position.y > screenHeight) {
    position.y = 0.;
  }
}

void Boid::update(const std::vector<Boid>& boids, double const delta_t) {
  updateVelocity(boids);
  updatePosition(delta_t);
  borders();
}

double Boid::angle(const sf::Vector2<double>& v) {
  return std::atan2(v.y, v.x);
}

}  // namespace bd