#ifndef COVARIANCES_H
#define COVARIANCES_H

#include <math.h>

namespace move
{

namespace Covariances
{

constexpr inline auto covarianceFrom(double cov)
{
  return cov * cov * 0.1;
}

constexpr auto xy{covarianceFrom(0.85)};
constexpr auto z{covarianceFrom(2.)};
constexpr auto rpy{covarianceFrom(0.8 * M_PI / 180)};
constexpr auto w{covarianceFrom(0.08 * M_PI/180)};
constexpr auto a{covarianceFrom(0.002 * 9.81)};

}

}

#endif // COVARIANCES_H
