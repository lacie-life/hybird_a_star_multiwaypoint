#include "rs_convert/PathSettings.hpp"
#include "rs_convert/PathSegmentLengths.hpp"
#include "rs_convert/SegmentSettings.hpp"
#include "rs_convert/RSCar.hpp"

namespace PathfindingForVehicles::ReedsSheppPaths
{

    std::vector<SegmentSettings> PathSettings::GetSettings(PathWords word, PathSegmentLengths pathLengths)
    {
        switch (word)
        {
        // 8.1: CSC, same turn
        case PathWords::Lf_Sf_Lf: return getLfSfLfPath(pathLengths);
        case PathWords::Lb_Sb_Lb: return TimeFlip(getLfSfLfPath(pathLengths));
        case PathWords::Rf_Sf_Rf: return Reflect(getLfSfLfPath(pathLengths));
        case PathWords::Rb_Sb_Rb: return Reflect(TimeFlip(getLfSfLfPath(pathLengths)));

        // 8.2: CSC, different turn
        case PathWords::Lf_Sf_Rf: return getLfSfRfPath(pathLengths);
        case PathWords::Lb_Sb_Rb: return TimeFlip(getLfSfRfPath(pathLengths));
        case PathWords::Rf_Sf_Lf: return Reflect(getLfSfRfPath(pathLengths));
        case PathWords::Rb_Sb_Lb: return Reflect(TimeFlip(getLfSfRfPath(pathLengths)));

        // 8.3: C|C|C
        case PathWords::Lf_Rb_Lf: return getLfRbLfPath(pathLengths);
        case PathWords::Lb_Rf_Lb: return TimeFlip(getLfRbLfPath(pathLengths));
        case PathWords::Rf_Lb_Rf: return Reflect(getLfRbLfPath(pathLengths));
        case PathWords::Rb_Lf_Rb: return Reflect(TimeFlip(getLfRbLfPath(pathLengths)));

        // 8.4: C|CC
        case PathWords::Lf_Rb_Lb: return getLfRbLbPath(pathLengths);
        case PathWords::Lb_Rf_Lf: return TimeFlip(getLfRbLbPath(pathLengths));
        case PathWords::Rf_Lb_Rb: return Reflect(getLfRbLbPath(pathLengths));
        case PathWords::Rb_Lf_Rf: return Reflect(TimeFlip(getLfRbLbPath(pathLengths)));

        // 8.4: CC|C
        case PathWords::Lf_Rf_Lb: return getLfRfLbPath(pathLengths);
        case PathWords::Lb_Rb_Lf: return TimeFlip(getLfRfLbPath(pathLengths));
        case PathWords::Rf_Lf_Rb: return Reflect(getLfRfLbPath(pathLengths));
        case PathWords::Rb_Lb_Rf: return Reflect(TimeFlip(getLfRfLbPath(pathLengths)));

        // 8.7: CCu|CuC
        case PathWords::Lf_Ruf_Lub_Rb: return getLfRufLubRbPath(pathLengths);
        case PathWords::Lb_Rub_Luf_Rf: return TimeFlip(getLfRufLubRbPath(pathLengths));
        case PathWords::Rf_Luf_Rub_Lb: return Reflect(getLfRufLubRbPath(pathLengths));
        case PathWords::Rb_Lub_Ruf_Lf: return Reflect(TimeFlip(getLfRufLubRbPath(pathLengths)));

        // 8.8: C|CuCu|C
        case PathWords::Lf_Rub_Lub_Rf: return getLfRubLubRfPath(pathLengths);
        case PathWords::Lb_Ruf_Luf_Rb: return TimeFlip(getLfRubLubRfPath(pathLengths));
        case PathWords::Rf_Lub_Rub_Lf: return Reflect(getLfRubLubRfPath(pathLengths));
        case PathWords::Rb_Luf_Ruf_Lb: return Reflect(TimeFlip(getLfRubLubRfPath(pathLengths)));

        // 8.9: C|C(pi/2)SC, same turn
        case PathWords::Lf_Rbpi2_Sb_Lb: return getLfRbpi2SbLbPath(pathLengths);
        case PathWords::Lb_Rfpi2_Sf_Lf: return TimeFlip(getLfRbpi2SbLbPath(pathLengths));
        case PathWords::Rf_Lbpi2_Sb_Rb: return Reflect(getLfRbpi2SbLbPath(pathLengths));
        case PathWords::Rb_Lfpi2_Sf_Rf: return Reflect(TimeFlip(getLfRbpi2SbLbPath(pathLengths)));

        // 8.10: C|C(pi/2)SC, different turn
        case PathWords::Lf_Rbpi2_Sb_Rb: return getLfRbpi2SbRbPath(pathLengths);
        case PathWords::Lb_Rfpi2_Sf_Rf: return TimeFlip(getLfRbpi2SbRbPath(pathLengths));
        case PathWords::Rf_Lbpi2_Sb_Lb: return Reflect(getLfRbpi2SbRbPath(pathLengths));
        case PathWords::Rb_Lfpi2_Sf_Lf: return Reflect(TimeFlip(getLfRbpi2SbRbPath(pathLengths)));

        // 8.9 (reversed): CSC(pi/2)|C, same turn
        case PathWords::Lf_Sf_Rfpi2_Lb: return getLfSfRfpi2LbPath(pathLengths);
        case PathWords::Lb_Sb_Rbpi2_Lf: return TimeFlip(getLfSfRfpi2LbPath(pathLengths));
        case PathWords::Rf_Sf_Lfpi2_Rb: return Reflect(getLfSfRfpi2LbPath(pathLengths));
        case PathWords::Rb_Sb_Lbpi2_Rf: return Reflect(TimeFlip(getLfSfRfpi2LbPath(pathLengths)));

        // 8.10 (reversed): CSC(pi/2)|C, different turn
        case PathWords::Lf_Sf_Lfpi2_Rb: return getLfSfLfpi2RbPath(pathLengths);
        case PathWords::Lb_Sb_Lbpi2_Rf: return TimeFlip(getLfSfLfpi2RbPath(pathLengths));
        case PathWords::Rf_Sf_Rfpi2_Lb: return Reflect(getLfSfLfpi2RbPath(pathLengths));
        case PathWords::Rb_Sb_Rbpi2_Lf: return Reflect(TimeFlip(getLfSfLfpi2RbPath(pathLengths)));

        // 8.11: C|C(pi/2)SC(pi/2)|C
        case PathWords::Lf_Rbpi2_Sb_Lbpi2_Rf: return getLfRbpi2SbLbpi2RfPath(pathLengths);
        case PathWords::Lb_Rfpi2_Sf_Lfpi2_Rb: return TimeFlip(getLfRbpi2SbLbpi2RfPath(pathLengths));
        case PathWords::Rf_Lbpi2_Sb_Rbpi2_Lf: return Reflect(getLfRbpi2SbLbpi2RfPath(pathLengths));
        case PathWords::Rb_Lfpi2_Sf_Rfpi2_Lb: return Reflect(TimeFlip(getLfRbpi2SbLbpi2RfPath(pathLengths)));
        }

        return std::vector<SegmentSettings>();
    }

    std::vector<SegmentSettings> PathSettings::TimeFlip(std::vector<SegmentSettings> pathSettings)
    {
        for (auto settings : pathSettings)
        {
            // Set it to forward
            RSCar::Gear flippedGear = RSCar::Gear::Forward;

            // If the current setting is forward, then flip
            if (settings.gear == RSCar::Gear::Forward)
            {
                flippedGear = RSCar::Gear::Back;
            }

            settings.gear = flippedGear;
        }

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::Reflect(std::vector<SegmentSettings> pathSettings)
    {
        for (auto settings : pathSettings)
        {
            // Ignore if w are going straight
            if (settings.steering == RSCar::Steering::Straight)
            {
                continue;
            }

            // Set it to right
            RSCar::Steering flippedSteering = RSCar::Steering::Right;

            // If the current setting is right, then flip
            if (settings.steering == RSCar::Steering::Right)
            {
                flippedSteering = RSCar::Steering::Left;
            }

            settings.steering = flippedSteering;
        }

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::getLfSfLfPath(PathSegmentLengths pathLength)
    {
        std::vector<SegmentSettings> pathSettings;

        SegmentSettings tempVar(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t);
        pathSettings.push_back(tempVar);
        SegmentSettings tempVar2(RSCar::Steering::Straight, RSCar::Gear::Forward, pathLength.u);
        pathSettings.push_back(tempVar2);
        SegmentSettings tempVar3(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.v);
        pathSettings.push_back(tempVar3);

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::getLfSfRfPath(PathSegmentLengths pathLength)
    {
        std::vector<SegmentSettings> pathSettings;

        SegmentSettings tempVar(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t);
        pathSettings.push_back(tempVar);
        SegmentSettings tempVar2(RSCar::Steering::Straight, RSCar::Gear::Forward, pathLength.u);
        pathSettings.push_back(tempVar2);
        SegmentSettings tempVar3(RSCar::Steering::Right, RSCar::Gear::Forward, pathLength.v);
        pathSettings.push_back(tempVar3);

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::getLfRbLfPath(PathSegmentLengths pathLength)
    {
        std::vector<SegmentSettings> pathSettings;

        SegmentSettings tempVar(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t);
        pathSettings.push_back(tempVar);
        SegmentSettings tempVar2(RSCar::Steering::Right, RSCar::Gear::Back, pathLength.u);
        pathSettings.push_back(tempVar2);
        SegmentSettings tempVar3(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.v);
        pathSettings.push_back(tempVar3);

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::getLfRbLbPath(PathSegmentLengths pathLength)
    {
        std::vector<SegmentSettings> pathSettings;

        SegmentSettings tempVar(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t);
        pathSettings.push_back(tempVar);
        SegmentSettings tempVar2(RSCar::Steering::Right, RSCar::Gear::Back, pathLength.u);
        pathSettings.push_back(tempVar2);
        SegmentSettings tempVar3(RSCar::Steering::Left, RSCar::Gear::Back, pathLength.v);
        pathSettings.push_back(tempVar3);

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::getLfRfLbPath(PathSegmentLengths pathLength)
    {
        std::vector<SegmentSettings> pathSettings;

        SegmentSettings tempVar(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t);
        pathSettings.push_back(tempVar);
        SegmentSettings tempVar2(RSCar::Steering::Right, RSCar::Gear::Forward, pathLength.u);
        pathSettings.push_back(tempVar2);
        SegmentSettings tempVar3(RSCar::Steering::Left, RSCar::Gear::Back, pathLength.v);
        pathSettings.push_back(tempVar3);

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::getLfRufLubRbPath(PathSegmentLengths pathLength)
    {
        std::vector<SegmentSettings> pathSettings;

        SegmentSettings tempVar(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t);
        pathSettings.push_back(tempVar);
        SegmentSettings tempVar2(RSCar::Steering::Right, RSCar::Gear::Forward, pathLength.u);
        pathSettings.push_back(tempVar2);
        SegmentSettings tempVar3(RSCar::Steering::Left, RSCar::Gear::Back, pathLength.u);
        pathSettings.push_back(tempVar3);
        SegmentSettings tempVar4(RSCar::Steering::Right, RSCar::Gear::Back, pathLength.v);
        pathSettings.push_back(tempVar4);

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::getLfRubLubRfPath(PathSegmentLengths pathLength)
    {
        std::vector<SegmentSettings> pathSettings;

        SegmentSettings tempVar(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t);
        pathSettings.push_back(tempVar);
        SegmentSettings tempVar2(RSCar::Steering::Right, RSCar::Gear::Back, pathLength.u);
        pathSettings.push_back(tempVar2);
        SegmentSettings tempVar3(RSCar::Steering::Left, RSCar::Gear::Back, pathLength.u);
        pathSettings.push_back(tempVar3);
        SegmentSettings tempVar4(RSCar::Steering::Right, RSCar::Gear::Forward, pathLength.v);
        pathSettings.push_back(tempVar4);

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::getLfRbpi2SbLbPath(PathSegmentLengths pathLength)
    {
        std::vector<SegmentSettings> pathSettings;

        SegmentSettings tempVar(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t);
        pathSettings.push_back(tempVar);
        SegmentSettings tempVar2(RSCar::Steering::Right, RSCar::Gear::Back, M_PI * 0.5);
        pathSettings.push_back(tempVar2);
        SegmentSettings tempVar3(RSCar::Steering::Straight, RSCar::Gear::Back, pathLength.u);
        pathSettings.push_back(tempVar3);
        SegmentSettings tempVar4(RSCar::Steering::Left, RSCar::Gear::Back, pathLength.v);
        pathSettings.push_back(tempVar4);

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::getLfRbpi2SbRbPath(PathSegmentLengths pathLength)
    {
        std::vector<SegmentSettings> pathSettings;

        SegmentSettings tempVar(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t);
        pathSettings.push_back(tempVar);
        SegmentSettings tempVar2(RSCar::Steering::Right, RSCar::Gear::Back, M_PI * 0.5);
        pathSettings.push_back(tempVar2);
        SegmentSettings tempVar3(RSCar::Steering::Straight, RSCar::Gear::Back, pathLength.u);
        pathSettings.push_back(tempVar3);
        SegmentSettings tempVar4(RSCar::Steering::Right, RSCar::Gear::Back, pathLength.v);
        pathSettings.push_back(tempVar4);

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::getLfSfRfpi2LbPath(PathSegmentLengths pathLength)
    {
        std::vector<SegmentSettings> pathSettings;

        SegmentSettings tempVar(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t);
        pathSettings.push_back(tempVar);
        SegmentSettings tempVar2(RSCar::Steering::Straight, RSCar::Gear::Forward, pathLength.u);
        pathSettings.push_back(tempVar2);
        SegmentSettings tempVar3(RSCar::Steering::Right, RSCar::Gear::Forward, M_PI * 0.5);
        pathSettings.push_back(tempVar3);
        SegmentSettings tempVar4(RSCar::Steering::Left, RSCar::Gear::Back, pathLength.v);
        pathSettings.push_back(tempVar4);

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::getLfSfLfpi2RbPath(PathSegmentLengths pathLength)
    {
        std::vector<SegmentSettings> pathSettings;

        SegmentSettings tempVar(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t);
        pathSettings.push_back(tempVar);
        SegmentSettings tempVar2(RSCar::Steering::Straight, RSCar::Gear::Forward, pathLength.u);
        pathSettings.push_back(tempVar2);
        SegmentSettings tempVar3(RSCar::Steering::Left, RSCar::Gear::Forward, M_PI * 0.5);
        pathSettings.push_back(tempVar3);
        SegmentSettings tempVar4(RSCar::Steering::Right, RSCar::Gear::Back, pathLength.v);
        pathSettings.push_back(tempVar4);

        return pathSettings;
    }

    std::vector<SegmentSettings> PathSettings::getLfRbpi2SbLbpi2RfPath(PathSegmentLengths pathLength)
    {
        std::vector<SegmentSettings> pathSettings;

        SegmentSettings tempVar(RSCar::Steering::Left, RSCar::Gear::Forward, pathLength.t);
        pathSettings.push_back(tempVar);
        SegmentSettings tempVar2(RSCar::Steering::Right, RSCar::Gear::Back, M_PI * 0.5);
        pathSettings.push_back(tempVar2);
        SegmentSettings tempVar3(RSCar::Steering::Straight, RSCar::Gear::Back, pathLength.u);
        pathSettings.push_back(tempVar3);
        SegmentSettings tempVar4(RSCar::Steering::Left, RSCar::Gear::Back, M_PI * 0.5);
        pathSettings.push_back(tempVar4);
        SegmentSettings tempVar5(RSCar::Steering::Right, RSCar::Gear::Forward, pathLength.v);
        pathSettings.push_back(tempVar5);

        return pathSettings;
    }
}
