
/*
    Implementation of the optimal path formulas given in the following paper:

    OPTIMAL PATHS FOR A CAR THAT GOES BOTH FORWARDS AND BACKWARDS
    J. A. REEDS AND L. A. SHEPP

    notes: there are some typos in the formulas given in the paper;

    some formulas have been adapted (cf http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c)

    Each of the 12 functions (each representing 4 of the 48 possible words)
    have 3 arguments x, y and phi, the goal position and angle (in degrees) of the
    object given it starts at position (0, 0) and angle 0, and returns the
    corresponding path (if it exists) as a list of PathElements (or an empty list).

    (actually there are less than 48 possible words but this code is not optimized)
*/

#include "rs_convert/PathLengthMath.hpp"
#include "rs_convert/PathSegmentLengths.hpp"
#include "rs_convert/RSCar.hpp"

namespace PathfindingForVehicles::ReedsSheppPaths
{

    double PathLengthMath::GetLength(RSCar car, PathWords word, PathSegmentLengths &pathLengths)
    {
        switch (word)
        {
        // 8.1: CSC, same turn
        case PathWords::Lf_Sf_Lf: return Lf_Sf_Lf(car, pathLengths); // xyz right, oz rotation
        case PathWords::Lb_Sb_Lb: return Lf_Sf_Lf(car.changeData(-car.getX(), car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rf_Sf_Rf: return Lf_Sf_Lf(car.changeData(car.getX(), -car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rb_Sb_Rb: return Lf_Sf_Lf(car.changeData(-car.getX(), -car.getY(), car.getHeadingInRad()), pathLengths);

        // 8.2: CSC, different turn
        case PathWords::Lf_Sf_Rf: return Lf_Sf_Rf(car, pathLengths);
        case PathWords::Lb_Sb_Rb: return Lf_Sf_Rf(car.changeData(-car.getX(), car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rf_Sf_Lf: return Lf_Sf_Rf(car.changeData(car.getX(), -car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rb_Sb_Lb: return Lf_Sf_Rf(car.changeData(-car.getX(), -car.getY(), car.getHeadingInRad()), pathLengths);

        // 8.3: C|C|C
        case PathWords::Lf_Rb_Lf: return Lf_Rb_Lf(car, pathLengths);
        case PathWords::Lb_Rf_Lb: return Lf_Rb_Lf(car.changeData(-car.getX(), car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rf_Lb_Rf: return Lf_Rb_Lf(car.changeData(car.getX(), -car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rb_Lf_Rb: return Lf_Rb_Lf(car.changeData(-car.getX(), -car.getY(), car.getHeadingInRad()), pathLengths);

        // 8.4: C|CC
        case PathWords::Lf_Rb_Lb: return Lf_Rb_Lb(car, pathLengths);
        case PathWords::Lb_Rf_Lf: return Lf_Rb_Lb(car.changeData(-car.getX(), car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rf_Lb_Rb: return Lf_Rb_Lb(car.changeData(car.getX(), -car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rb_Lf_Rf: return Lf_Rb_Lb(car.changeData(-car.getX(), -car.getY(), car.getHeadingInRad()), pathLengths);

        // 8.4: CC|C
        case PathWords::Lf_Rf_Lb: return Lf_Rf_Lb(car, pathLengths);
        case PathWords::Lb_Rb_Lf: return Lf_Rf_Lb(car.changeData(-car.getX(), car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rf_Lf_Rb: return Lf_Rf_Lb(car.changeData(car.getX(), -car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rb_Lb_Rf: return Lf_Rf_Lb(car.changeData(-car.getX(), -car.getY(), car.getHeadingInRad()), pathLengths);

        // 8.7: CCu|CuC
        case PathWords::Lf_Ruf_Lub_Rb: return Lf_Ruf_Lub_Rb(car, pathLengths);
        case PathWords::Lb_Rub_Luf_Rf: return Lf_Ruf_Lub_Rb(car.changeData(-car.getX(), car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rf_Luf_Rub_Lb: return Lf_Ruf_Lub_Rb(car.changeData(car.getX(), -car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rb_Lub_Ruf_Lf: return Lf_Ruf_Lub_Rb(car.changeData(-car.getX(), -car.getY(), car.getHeadingInRad()), pathLengths);

        // 8.8: C|CuCu|C
        case PathWords::Lf_Rub_Lub_Rf: return Lf_Rub_Lub_Rf(car, pathLengths);
        case PathWords::Lb_Ruf_Luf_Rb: return Lf_Rub_Lub_Rf(car.changeData(-car.getX(), car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rf_Lub_Rub_Lf: return Lf_Rub_Lub_Rf(car.changeData(car.getX(), -car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rb_Luf_Ruf_Lb: return Lf_Rub_Lub_Rf(car.changeData(-car.getX(), -car.getY(), car.getHeadingInRad()), pathLengths);

        // 8.9: C|C(pi/2)SC, same turn
        case PathWords::Lf_Rbpi2_Sb_Lb: return Lf_Rbpi2_Sb_Lb(car, pathLengths);
        case PathWords::Lb_Rfpi2_Sf_Lf: return Lf_Rbpi2_Sb_Lb(car.changeData(-car.getX(), car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rf_Lbpi2_Sb_Rb: return Lf_Rbpi2_Sb_Lb(car.changeData(car.getX(), -car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rb_Lfpi2_Sf_Rf: return Lf_Rbpi2_Sb_Lb(car.changeData(-car.getX(), -car.getY(), car.getHeadingInRad()), pathLengths);

        // 8.10: C|C(pi/2)SC, different turn
        case PathWords::Lf_Rbpi2_Sb_Rb: return Lf_Rbpi2_Sb_Rb(car, pathLengths);
        case PathWords::Lb_Rfpi2_Sf_Rf: return Lf_Rbpi2_Sb_Rb(car.changeData(-car.getX(), car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rf_Lbpi2_Sb_Lb: return Lf_Rbpi2_Sb_Rb(car.changeData(car.getX(), -car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rb_Lfpi2_Sf_Lf: return Lf_Rbpi2_Sb_Rb(car.changeData(-car.getX(), -car.getY(), car.getHeadingInRad()), pathLengths);

        // 8.9(reversed): CSC(pi / 2) | C, same turn
        case PathWords::Lf_Sf_Rfpi2_Lb: return Lf_Sf_Rfpi2_Lb(car, pathLengths);
        case PathWords::Lb_Sb_Rbpi2_Lf: return Lf_Sf_Rfpi2_Lb(car.changeData(-car.getX(), car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rf_Sf_Lfpi2_Rb: return Lf_Sf_Rfpi2_Lb(car.changeData(car.getX(), -car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rb_Sb_Lbpi2_Rf: return Lf_Sf_Rfpi2_Lb(car.changeData(-car.getX(), -car.getY(), car.getHeadingInRad()), pathLengths);

        // 8.10 (reversed): CSC(pi/2)|C, different turn
        case PathWords::Lf_Sf_Lfpi2_Rb: return Lf_Sf_Lfpi2_Rb(car, pathLengths);
        case PathWords::Lb_Sb_Lbpi2_Rf: return Lf_Sf_Lfpi2_Rb(car.changeData(-car.getX(), car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rf_Sf_Rfpi2_Lb: return Lf_Sf_Lfpi2_Rb(car.changeData(car.getX(), -car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rb_Sb_Rbpi2_Lf: return Lf_Sf_Lfpi2_Rb(car.changeData(-car.getX(), -car.getY(), car.getHeadingInRad()), pathLengths);

        // 8.11: C | C(pi / 2)SC(pi / 2) | C
        case PathWords::Lf_Rbpi2_Sb_Lbpi2_Rf: return Lf_Rbpi2_Sb_Lbpi2_Rf(car, pathLengths);
        case PathWords::Lb_Rfpi2_Sf_Lfpi2_Rb: return Lf_Rbpi2_Sb_Lbpi2_Rf(car.changeData(-car.getX(), car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rf_Lbpi2_Sb_Rbpi2_Lf: return Lf_Rbpi2_Sb_Lbpi2_Rf(car.changeData(car.getX(), -car.getY(), -car.getHeadingInRad()), pathLengths);
        case PathWords::Rb_Lfpi2_Sf_Rfpi2_Lb: return Lf_Rbpi2_Sb_Lbpi2_Rf(car.changeData(-car.getX(), -car.getY(), car.getHeadingInRad()), pathLengths);

        default:
            pathLengths = PathSegmentLengths(0.0, 0.0, 0.0); break;
        }

        return std::numeric_limits<double>::max();
    }

    double PathLengthMath::Lf_Sf_Lf(RSCar goal, PathSegmentLengths &path)
    {
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        path = PathSegmentLengths(0.0, 0.0, 0.0);

        double x = goal.getX();
        double y = goal.getY();
        double phi = goal.getHeadingInRad();

        // Calculations
        // On page 24 in the Reeds-Shepp report, it says:
        //(u, t) = R(x - sin(phi), y - 1 + cos(phi))
        // v = M(phi - t)
        // L = |t| + |u| + |v|
        // This path can't be optimal if t or v is outside [0, pi]
        R(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);

        v = mrpt::math::wrapToPi<double>(phi - t);

        // Debug.Log(t + " " + u + " " + v);

        // Dont need to check u because its straight
        if (isCurveSegmentInvalid(t) || isCurveSegmentInvalid(v))
        {
            return std::numeric_limits<double>::max();
        }

        path.t = t;
        path.u = u;
        path.v = v;

        double totalLength = t + u + v;

        return totalLength;
    }

    double PathLengthMath::Lf_Sf_Rf(RSCar goal, PathSegmentLengths &path)
    {
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        path = PathSegmentLengths(0.0, 0.0, 0.0);

        double x = goal.getX();
        double y = goal.getY();
        double phi = goal.getHeadingInRad();

        // Calculations
        double u1, t1;

        R(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);

        if (u1 * u1 < 4.0)
        {
            return std::numeric_limits<double>::max();
        }

        u = std::sqrt((u1 * u1) - 4.0);

        double T, theta;

        R(u, 2.0, T, theta);

        t = mrpt::math::wrapToPi<double>(t1 + theta);

        v = mrpt::math::wrapToPi<double>(t - phi);

        // Debug.Log(t + " " + u + " " + v);

        if (isCurveSegmentInvalid(t) || isCurveSegmentInvalid(v))
        {
            return std::numeric_limits<double>::max();
        }

        path.t = t;
        path.u = u;
        path.v = v;

        double totalLength = t + u + v;

        return totalLength;
    }

    // 8.3
    double PathLengthMath::Lf_Rb_Lf(RSCar goal, PathSegmentLengths &path)
    {
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        path = PathSegmentLengths(0.0, 0.0, 0.0);

        double x = goal.getX();
        double y = goal.getY();
        double phi = goal.getHeadingInRad();

        // Calculations
        // Uses a modified formula adapted from the c_c_c function from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
        double xi = x - std::sin(phi);
        double eta = y - 1.0 + std::cos(phi);

        double u1, theta;

        R(xi, eta, u1, theta);

        if (u1 > 4.0)
        {
            return std::numeric_limits<double>::max();
        }

        double alpha = std::acos(u1 / 4.0);

        t = mrpt::math::wrapTo2Pi<double>(HALF_PI + alpha + theta);
        u = mrpt::math::wrapTo2Pi<double>(M_PI - 2.0 * alpha);
        v = mrpt::math::wrapTo2Pi<double>(phi - t - u);

        // Check all 3 curves
        if (isCurveSegmentInvalid(t) || isCurveSegmentInvalid(u) || isCurveSegmentInvalid(v))
        {
            return std::numeric_limits<double>::max();
        }

        path.t = t;
        path.u = u;
        path.v = v;

        double totalLength = t + u + v;

        return totalLength;
    }

    // Formula 8.4 (1): C|CC
    double PathLengthMath::Lf_Rb_Lb(RSCar goal, PathSegmentLengths &path)
    {
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        path = PathSegmentLengths(0.0, 0.0, 0.0);

        double x = goal.getX();
        double y = goal.getY();
        double phi = goal.getHeadingInRad();

        // Calculations
        // Uses a modified formula adapted from the c_cc function from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
        double xi = x - std::sin(phi);
        double eta = y - 1.0 + std::cos(phi);

        double u1, theta;

        R(xi, eta, u1, theta);

        if (u1 > 4.0)
        {
            return std::numeric_limits<double>::max();
        }

        double alpha = std::acos(u1 / 4.0);

        t = mrpt::math::wrapTo2Pi<double>(HALF_PI + alpha + theta);
        u = mrpt::math::wrapTo2Pi<double>(M_PI - 2.0 * alpha);
        // This part is the only thing thats different from 8.3
        v = mrpt::math::wrapTo2Pi<double>(t + u - phi);

        // Check all 3 curves
        if (isCurveSegmentInvalid(t) || isCurveSegmentInvalid(u) || isCurveSegmentInvalid(v))
        {
            return std::numeric_limits<double>::max();
        }

        path.t = t;
        path.u = u;
        path.v = v;

        double totalLength = t + u + v;

        return totalLength;
    }

    // Formula 8.4 (2): CC|C
    double PathLengthMath::Lf_Rf_Lb(RSCar goal, PathSegmentLengths &path)
    {
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        path = PathSegmentLengths(0.0, 0.0, 0.0);

        double x = goal.getX();
        double y = goal.getY();
        double phi = goal.getHeadingInRad();

        // Calculations
        // Uses a modified formula adapted from the cc_c function from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
        double xi = x - std::sin(phi);
        double eta = y - 1.0 + std::cos(phi);

        double u1, theta;

        R(xi, eta, u1, theta);

        if (u1 > 4.0)
        {
            return std::numeric_limits<double>::max();
        }

        u = std::acos((8.0 - (u1 * u1)) / 8.0);

        double va = std::sin(u);

        double alpha = std::asin(2.0 * va / u1);

        t = mrpt::math::wrapTo2Pi<double>(HALF_PI - alpha + theta);
        v = mrpt::math::wrapTo2Pi<double>(t - u - phi);

        // Check all 3 curves
        if (isCurveSegmentInvalid(t) || isCurveSegmentInvalid(u) || isCurveSegmentInvalid(v))
        {
            return std::numeric_limits<double>::max();
        }

        path.t = t;
        path.u = u;
        path.v = v;

        double totalLength = t + u + v;

        return totalLength;
    }

    // Formula 8.7: CCu|CuC
    double PathLengthMath::Lf_Ruf_Lub_Rb(RSCar goal, PathSegmentLengths &path)
    {
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        path = PathSegmentLengths(0.0, 0.0, 0.0);

        double x = goal.getX();
        double y = goal.getY();
        double phi = goal.getHeadingInRad();

        // Calculations
        // Uses a modified formula adapted from the ccu_cuc function from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
        double xi = x + std::sin(phi);
        double eta = y - 1.0 - std::cos(phi);

        double u1, theta;

        R(xi, eta, u1, theta);

        if (u1 > 4.0)
        {
            return std::numeric_limits<double>::max();
        }

        if (u1 > 2.0)
        {
            double alpha = std::acos((u1 - 2.0) / 4.0);

            t = mrpt::math::wrapTo2Pi<double>(HALF_PI + theta - alpha);
            u = mrpt::math::wrapTo2Pi<double>(M_PI - alpha);
            v = mrpt::math::wrapTo2Pi<double>(phi - t + 2.0 * u);
        }
        else
        {
            double alpha = std::acos((u1 + 2.0) / 4.0);

            t = mrpt::math::wrapTo2Pi<double>(HALF_PI + theta + alpha);
            u = mrpt::math::wrapTo2Pi<double>(alpha);
            v = mrpt::math::wrapTo2Pi<double>(phi - t + 2.0 * u);
        }

        // Check all 3 curves
        if (isCurveSegmentInvalid(t) || isCurveSegmentInvalid(u) || isCurveSegmentInvalid(v))
        {
            return std::numeric_limits<double>::max();
        }

        path.t = t;
        path.u = u;
        path.v = v;

        double totalLength = t + u + u + v;

        return totalLength;
    }

    // Formula 8.8: C|CuCu|C
    double PathLengthMath::Lf_Rub_Lub_Rf(RSCar goal, PathSegmentLengths &path)
    {
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        path = PathSegmentLengths(0.0, 0.0, 0.0);

        double x = goal.getX();
        double y = goal.getY();
        double phi = goal.getHeadingInRad();

        // Calculations
        // Uses a modified formula adapted from the c_cucu_c function from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
        double xi = x + std::sin(phi);
        double eta = y - 1.0 - std::cos(phi);

        double u1, theta;

        R(xi, eta, u1, theta);

        if (u1 > 6.0)
        {
            return std::numeric_limits<double>::max();
        }

        // This is the part thats different from the original report:
        double va1 = (20.0 - u1 * u1) / 16.0;

        if (va1 < 0.0 || va1 > 1.0)
        {
            return std::numeric_limits<double>::max();
        }

        u = std::acos(va1);

        double va2 = std::sin(u);

        double alpha = std::asin((2.0 * va2) / u1);

        t = mrpt::math::wrapTo2Pi<double>(HALF_PI + theta + alpha);
        v = mrpt::math::wrapTo2Pi<double>(t - phi);

        // Check all 3 curves
        if (isCurveSegmentInvalid(t) || isCurveSegmentInvalid(u) || isCurveSegmentInvalid(v))
        {
            return std::numeric_limits<double>::max();
        }

        path.t = t;
        path.u = u;
        path.v = v;

        double totalLength = t + u + u + v;

        return totalLength;
    }

    // Formula 8.9 (1): C|C[pi/2]SC
    double PathLengthMath::Lf_Rbpi2_Sb_Lb(RSCar goal, PathSegmentLengths &path)
    {
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        path = PathSegmentLengths(0.0, 0.0, 0.0);

        double x = goal.getX();
        double y = goal.getY();
        double phi = goal.getHeadingInRad();

        // Calculations
        // Uses a modified formula adapted from the c_c2sca function from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
        double xi = x - std::sin(phi);
        double eta = y - 1.0 + std::cos(phi);

        double u1, theta;

        R(xi, eta, u1, theta);

        double u1Squared = u1 * u1;

        if (u1Squared < 4.0)
        {
            return std::numeric_limits<double>::max();
        }

        // This is the part that is different from the original report:
        double va1 = (20 - u1 * u1) / 16.0;

        if (va1 < 0.0 || va1 > 1.0)
        {
            return std::numeric_limits<double>::max();
        }

        u = std::sqrt(u1Squared - 4.0) - 2.0;

        if (u < 0.0)
        {
            return std::numeric_limits<double>::max();
        }

        double alpha = std::atan2(2.0, u + 2.0);

        t = mrpt::math::wrapTo2Pi<double>(HALF_PI + theta + alpha);
        v = mrpt::math::wrapTo2Pi<double>(t + HALF_PI - phi);

        // Check all 2 curves (pi/2 is always a valid curve)
        if (isCurveSegmentInvalid(t) || isCurveSegmentInvalid(v))
        {
            return std::numeric_limits<double>::max();
        }

        path.t = t;
        path.u = u;
        path.v = v;

        double totalLength = t + HALF_PI + u + v;

        return totalLength;
    }

    // Formula 8.9 (2): CSC[pi/2]|C
    double PathLengthMath::Lf_Sf_Rfpi2_Lb(RSCar goal, PathSegmentLengths &path)
    {
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        path = PathSegmentLengths(0.0, 0.0, 0.0);

        double x = goal.getX();
        double y = goal.getY();
        double phi = goal.getHeadingInRad();

        // Calculations
        // Uses a modified formula adapted from the csc2_ca function from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
        double xi = x - std::sin(phi);
        double eta = y - 1.0 + std::cos(phi);

        double u1, theta;

        R(xi, eta, u1, theta);

        double u1Squared = u1 * u1;

        if (u1 < 2.0)
        {
            return std::numeric_limits<double>::max();
        }

        // This is the part thats different from the original report:
        u = std::sqrt(u1Squared - 4.0) - 2.0;

        if (u < 0.0)
        {
            return std::numeric_limits<double>::max();
        }

        double alpha = std::atan2(u + 2.0, 2.0);

        t = mrpt::math::wrapTo2Pi<double>(HALF_PI + theta - alpha);
        v = mrpt::math::wrapTo2Pi<double>(t - HALF_PI - phi);

        // Check all 2 curves
        if (isCurveSegmentInvalid(t) || isCurveSegmentInvalid(v))
        {
            return std::numeric_limits<double>::max();
        }

        path.t = t;
        path.u = u;
        path.v = v;

        double totalLength = t + u + HALF_PI + v;

        return totalLength;
    }

    // Formula 8.10 (1): C|C[pi/2]SC
    double PathLengthMath::Lf_Rbpi2_Sb_Rb(RSCar goal, PathSegmentLengths &path)
    {
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        path = PathSegmentLengths(0.0, 0.0, 0.0);

        double x = goal.getX();
        double y = goal.getY();
        double phi = goal.getHeadingInRad();

        // Calculations
        // Uses a modified formula adapted from the c_c2scb function from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
        double xi = x + std::sin(phi);
        double eta = y - 1.0 - std::cos(phi);

        double u1, theta;

        R(xi, eta, u1, theta);

        if (u1 < 2.0)
        {
            return std::numeric_limits<double>::max();
        }

        // This is the part thats different from the original report:
        t = mrpt::math::wrapTo2Pi<double>(HALF_PI + theta);
        u = u1 - 2.0;
        v = mrpt::math::wrapTo2Pi<double>(phi - t - HALF_PI);

        // Check all 2 curves
        if (isCurveSegmentInvalid(t) || isCurveSegmentInvalid(v))
        {
            return std::numeric_limits<double>::max();
        }

        path.t = t;
        path.u = u;
        path.v = v;

        double totalLength = t + HALF_PI + u + v;

        return totalLength;
    }


    // Formula 8.10 (2): CSC[pi/2]|C
    double PathLengthMath::Lf_Sf_Lfpi2_Rb(RSCar goal, PathSegmentLengths &path)
    {
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        path = PathSegmentLengths(0.0, 0.0, 0.0);

        double x = goal.getX();
        double y = goal.getY();
        double phi = goal.getHeadingInRad();

        // Calculations
        // Uses a modified formula adapted from the csc2_cb function from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
        double xi = x + std::sin(phi);
        double eta = y - 1.0 - std::cos(phi);

        double u1, theta;

        R(xi, eta, u1, theta);

        if (u1 < 2.0)
        {
            return std::numeric_limits<double>::max();
        }

        // This is the part thats different from the original report:
        t = mrpt::math::wrapTo2Pi<double>(theta);
        u = u1 - 2.0;
        v = mrpt::math::wrapTo2Pi<double>(-t - HALF_PI + phi);

        // Check all 2 curves
        if (isCurveSegmentInvalid(t) || isCurveSegmentInvalid(v))
        {
            return std::numeric_limits<double>::max();
        }

        path.t = t;
        path.u = u;
        path.v = v;

        double totalLength = t + u + HALF_PI + v;

        return totalLength;
    }

    //  Formula 8.11: C|C[pi/2]SC[pi/2]|C
    double PathLengthMath::Lf_Rbpi2_Sb_Lbpi2_Rf(RSCar goal, PathSegmentLengths &path)
    {
        double t = 0.0;
        double u = 0.0;
        double v = 0.0;
        path = PathSegmentLengths(0.0, 0.0, 0.0);

        double x = goal.getX();
        double y = goal.getY();
        double phi = goal.getHeadingInRad();

        // Calculations
        // Uses a modified formula adapted from the c_c2sc2_c function from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
        double xi = x + std::sin(phi);
        double eta = y - 1.0 - std::cos(phi);

        double u1, theta;

        R(xi, eta, u1, theta);

        double u1Squared = u1 * u1;

        if (u1 < 4.0)
        {
            return std::numeric_limits<double>::max();
        }

        // This is the part thats different from the original report:
        u = std::sqrt(u1Squared - 4.0) - 4.0;

        if (u < 0.0)
        {
            return std::numeric_limits<double>::max();
        }

        double alpha = std::atan2(2.0, u + 4.0);

        t = mrpt::math::wrapTo2Pi<double>(HALF_PI + theta + alpha);
        v = mrpt::math::wrapTo2Pi<double>(t - phi);

        // Check all 2 curves
        if (isCurveSegmentInvalid(t) || isCurveSegmentInvalid(v))
        {
            return std::numeric_limits<double>::max();
        }

        path.t = t;
        path.u = u;
        path.v = v;

        double totalLength = t + HALF_PI + u + HALF_PI + v;

        return totalLength;
    }

    bool PathLengthMath::isCurveSegmentInvalid(double segmentLength)
    {
        // Is invalid if outside [0, pi]
        return segmentLength < 0.0 || segmentLength > PI;
    }

    double PathLengthMath::M(double angle)
    {
        angle = static_cast<double>(std::remainder(static_cast<double>(angle), static_cast<double>(TWO_PI)));

        if (angle <= -PI)
        {
            angle += TWO_PI;
        }
        else if (angle > PI)
        {
            angle -= TWO_PI;
        }

        return angle;

        // double a = std::fmod(angle + M_PI, 2.0 * M_PI);
        // if (a < 0.0)
        // {
        //     a += (2.0 * M_PI);
        // }
        // return a - M_PI;
    }

    double PathLengthMath::WrapAngleInRadians(double angle)
    {
        return M(angle);
    }

    void PathLengthMath::R(double x, double y, double &radius, double &angle)
    {
        // Radius
        radius = std::sqrt(x * x + y * y);
        // Angle
        angle = std::atan2(y, x);
    }
}
