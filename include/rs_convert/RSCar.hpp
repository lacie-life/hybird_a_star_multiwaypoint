#pragma once

#include <iostream>
#include <cmath>
#include "global/cvector_3d.hpp"

namespace PathfindingForVehicles::ReedsSheppPaths
{
    // Vehicle data needed to generated fixed paths
    class RSCar
    {
        // Left, Right, Straight
    public:
        enum class Steering
        {
            Left,
            Right,
            Straight
        };
        // Forward, Back
    public:
        enum class Gear
        {
            Forward,
            Back
        };

    public:
        RSCar::Steering steering = static_cast<RSCar::Steering>(0);

        RSCar::Gear gear = static_cast<RSCar::Gear>(0);

        CVector3d pos;

        // Should be in radians
    private:
        double heading = 0;

    public:
        virtual ~RSCar()
        {
        }

        RSCar(CVector3d &pos, double headingInRadians);

        RSCar(CVector3d &pos, double headingInRadians, Gear gear, Steering steering);

        // Copy data from car to this car
        RSCar(const RSCar &car);

        // Change car data
        RSCar changeData(double newXPos, double newYPos, double newHeading);

        //
        // Getters
        //
        double getHeadingInDegrees() const;

        double getHeadingInRad() const;

        double getX() const;

        double getY() const;

        double radToDeg(double radian) const
        {
            return (radian * (180 / M_PI));
        }

        void printData() const
        {
            std::cout << "pos.x " << pos.x << " pos.y " << pos.y << " heading " << heading << std::endl;
        }
    };
}
