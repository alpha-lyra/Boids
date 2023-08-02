#pragma once
#ifndef FLOCK_HPP
#define FLOCK_HPP

#include "boid.hpp"

namespace bd {

class Flock {
  std::vector<Boid> m_flock;

 public:
  Flock flock();

  int size() const { return m_flock.size(); }

  Boid getBoid(int i) const;
  Boid& getBoid(int i);

  void addBoid(const Boid& b);

  void updateFlock(double delta_t);

  double average_distance();

  double average_speed();

  void setParameters(const Parameters& par1);
};

}  // namespace bd

#endif