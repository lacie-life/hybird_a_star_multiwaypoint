#include "rs_convert/RSCar.hpp"
#include "rs_convert/PathLengthMath.hpp"

namespace PathfindingForVehicles::ReedsSheppPaths
{

    RSCar::RSCar(CVector3d &pos, double headingInRadians)
    {
        this->pos = pos;
        this->heading = ReedsSheppPaths::PathLengthMath::M(headingInRadians);
    }

    RSCar::RSCar(CVector3d &pos, double headingInRadians, Gear gear, Steering steering) : RSCar(pos, headingInRadians)
    {
        this->gear = gear;
        this->steering = steering;
    }

    RSCar::RSCar(const RSCar &car)
    {
        this->pos = car.pos;
        this->heading = car.heading;

        this->gear = car.gear;
        this->steering = car.steering;
    }

    RSCar RSCar::changeData(double newXPos, double newYPos, double newHeading)
    {
        CVector3d tempVar(newXPos, newYPos, pos.z );
        RSCar carCopy = RSCar(tempVar, newHeading);

        return carCopy;
    }

    double RSCar::getHeadingInDegrees() const
    {
        return radToDeg(heading);
    }

    double RSCar::getHeadingInRad() const
    {
        return heading;
    }

    double RSCar::getX() const
    {
        return pos.x;
    }

    double RSCar::getY() const
    {
        return pos.y;
    }
}
