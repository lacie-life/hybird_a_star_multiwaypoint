#include "rs_convert/SegmentSettings.hpp"



namespace PathfindingForVehicles::ReedsSheppPaths
{

	SegmentSettings::SegmentSettings(RSCar::Steering steering, RSCar::Gear gear, double length)
	{
		this->steering = steering;
		this->gear = gear;
		this->length = length;
	}
}
