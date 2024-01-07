#pragma once

#include "rs_convert/RSCar.hpp"

// C# TO C++ CONVERTER NOTE: Forward class declarations:
namespace PathfindingForVehicles::ReedsSheppPaths
{
    class RSCar;
}

namespace PathfindingForVehicles::ReedsSheppPaths
{
    // The settings the car should have to complete this path segment
    // Each Reeds-Shepp path consists of 3-5 of these segments
    class SegmentSettings
    {
        // The total length of this segment
    public:
        double length = 0;

        // Which way are we steering in this segment?
        RSCar::Steering steering = static_cast<RSCar::Steering>(0);

        // Are we driwing forward or reverse?
        RSCar::Gear gear = static_cast<RSCar::Gear>(0);

        SegmentSettings(RSCar::Steering steering, RSCar::Gear gear, double length);
    };
}
