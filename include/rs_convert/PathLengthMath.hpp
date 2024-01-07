#pragma once

#include "PathWords.hpp"
#include "global/wrap2pi.hpp"
#include <cmath>
#include <limits>

// C# TO C++ CONVERTER NOTE: Forward class declarations:
namespace PathfindingForVehicles::ReedsSheppPaths
{
    class PathSegmentLengths;
}
namespace PathfindingForVehicles::ReedsSheppPaths
{
    class RSCar;
}



namespace PathfindingForVehicles::ReedsSheppPaths
{
    // Calculate how long a given reeds-shepp path is
    class PathLengthMath final
    {
    private:
        static constexpr double HALF_PI = M_PI * 0.5;
        static constexpr double PI = M_PI;
        static constexpr double TWO_PI = M_PI * 2.0;

        // Could maybe optimize sin(phi) and cos(phi) because they are always the same

    public:
        static double GetLength(RSCar car, PathWords word, PathSegmentLengths &pathLengths);

        //
        // Length calculations for the different paths
        //

        // Basic idea (from "Optimal paths for a car that goes both forwards and backwards"):

        // In each formula the objective is to move from (0, 0, 0) to (x, y, phi)

        // We write (r, theta) = R(x, y) for the polar transform:
        //  r * cos(theta) = x
        //  r * sin(theta) = y
        //  r >= 0
        //  -pi <= theta < pi
        // So R(x, y) means that we translate x and y to polar coordinates from the cartesian
        // https://en.wikipedia.org/wiki/Polar_coordinate_system

        // We write phi = M(theta) if phi ≡ theta mod(2*pi) and -pi <= phi < pi, so M(theta) means modulus 2*pi

        // L is the overall length. We say L = ∞ if there's no solution

        // t, u, v are the unknown segment lengths
        // A Reeds-Shepp path consists of 3-5 segments, but only 3 segments have unpredetermined lengths
        // The other segments have each a length of a curve where we drive a length of pi/2
        // or the same length as another segment in the same path
        // The unit of length for a straight segment is whatever we want. For a circular arc it is in radians.

        // 8.1: CSC, same turn
        static double Lf_Sf_Lf(RSCar goal, PathSegmentLengths &path);

        // 8.2: CSC, different turn
        static double Lf_Sf_Rf(RSCar goal, PathSegmentLengths &path);

        // 8.3: C|C|C
        static double Lf_Rb_Lf(RSCar goal, PathSegmentLengths &path);

        // 8.4: C|CC
        static double Lf_Rb_Lb(RSCar goal, PathSegmentLengths &path);

        // 8.4: CC|C
        static double Lf_Rf_Lb(RSCar goal, PathSegmentLengths &path);

        // 8.7: CCu|CuC
        static double Lf_Ruf_Lub_Rb(RSCar goal, PathSegmentLengths &path);

        // 8.8: C|CuCu|C
        static double Lf_Rub_Lub_Rf(RSCar goal, PathSegmentLengths &path);

        // 8.9: C|C(pi/2)SC, same turn
        static double Lf_Rbpi2_Sb_Lb(RSCar goal, PathSegmentLengths &path);

        // 8.10: C|C(pi/2)SC, different turn
        static double Lf_Rbpi2_Sb_Rb(RSCar goal, PathSegmentLengths &path);

        // 8.9 (reversed): CSC(pi/2)|C, same turn
        static double Lf_Sf_Rfpi2_Lb(RSCar goal, PathSegmentLengths &path);

        // 8.10 (reversed): CSC(pi/2)|C, different turn
        static double Lf_Sf_Lfpi2_Rb(RSCar goal, PathSegmentLengths &path);

        // 8.11: C|C(pi/2)SC(pi/2)|C
        static double Lf_Rbpi2_Sb_Lbpi2_Rf(RSCar goal, PathSegmentLengths &path);

        //
        // Help methods
        //
    private:
        static bool isCurveSegmentInvalid(double segmentLength);

        // Wrap angle in radians, is called M in the report
        // http://www.technologicalutopia.com/sourcecode/xnageometry/mathhelper.cs.htm
    public:
        static double M(double angle);

        static double WrapAngleInRadians(double angle);

        // From cartesian to polar coordinates, is called R in the report
    private:
        static void R(double x, double y, double &radius, double &angle);
    };
}
