#ifndef GPS2ENU_H
#define GPS2ENU_H

#include <GeographicLib/UTMUPS.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>

using namespace sensor_msgs::msg;
using namespace geometry_msgs::msg;

namespace move
{

struct toENU
{
  double X0,Y0,Z0;
  int zone{-1};

  inline void printZone(int zone, double latitude, double longitude) const
  {
    std::cout << "Zone @ (" << latitude << ", " << longitude << ") is #"
              << zone << std::endl;
  }

  inline bool isInit() const
  {
    return zone > 0;
  }

  inline void setReference(const sensor_msgs::msg::NavSatFix &gps)
  {
    bool northp{true};
    GeographicLib::UTMUPS::Forward(gps.latitude, gps.longitude,
                                   zone, northp, X0,Y0);
    Z0 = gps.altitude - 1.62;
  }

  inline void transform(double latitude, double longitude, double altitude,
                        Point &p) const
  {
    bool northp{true};
    auto zone{this->zone};
    // this will throw if we move too far from the initial UTM zone
    GeographicLib::UTMUPS::Forward(latitude, longitude,
                                   zone, northp, p.x, p.y, this->zone);

    p.x -= X0;
    p.y -= Y0;
    p.z = altitude-Z0;
  }
};

}

#endif // GPS2ENU_H
