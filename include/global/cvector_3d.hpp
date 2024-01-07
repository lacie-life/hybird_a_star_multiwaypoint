#pragma once

#include <iostream>
#include <cmath>

namespace PathfindingForVehicles::ReedsSheppPaths
{

class CVector3d;

// ##############################################//
// 	Class for control point 3d	//
// ##############################################//
class CVector3d
{
public:
    double x;
    double y;
    double z;

public:
    /***** Constructor & Destructor *****/
    CVector3d();                                    // [01] Constructor
    CVector3d(double X, double Y, double Z);        // [02] Constructor
    ~CVector3d();                                   // [03] Destructor

    /***** Element relation *****/
    void Set(double X, double Y, double Z);         // [04] Set coordinates
    void UnitVector(double X, double Y, double Z);  // [05] After setting the coordinates, normalize
    void Clear();                                   // [06] Clear vector
    void Print();                                   // [07] Vector output

    /***** Overload *****/
    CVector3d &operator=(const CVector3d &obj);                       // [10] = operator overload
    CVector3d operator+(const CVector3d &obj);                        // [11] + operator overload
    CVector3d &operator+=(const CVector3d &obj);                      // [12] += operator overload
    CVector3d operator-(const CVector3d &obj);                        // [13] -operator overload
    CVector3d &operator-=(const CVector3d &obj);                      // [14] -= operator overload
    CVector3d operator-();                                            // [15] -operator overload
    CVector3d operator*(const double k);                              // [16] * operator overload
    friend CVector3d operator*(const double k, const CVector3d &obj); // [17] * operator overload

    CVector3d operator/(const double k);                              // [19] / operator overload
    operator double *();                                              // [20] * operator overload

    /***** vector calculation *****/
    CVector3d Mid(CVector3d &obj);                          // [20] Find the midpoint with the obj point
    CVector3d Normalize();                                  // [21] Normalize the stored coordinates
    void Normalize(CVector3d &obj);                         // [22] Normalize obj points
    double Norm();                                          // [23] Calculate the magnitude of the vector
    double Distance(CVector3d &obj);                        // [24] Find the distance to the obj point
    double SquaredDistance(CVector3d &obj);

    double Dot(CVector3d &obj);                             // [25] Calculate the inner product with the obj point
    CVector3d Cross(CVector3d &obj);                        // [26] Find the cross product with the obj point
    double Angle(CVector3d &obj);                           // [27] Angle with obj
    double PointLineDistance(CVector3d &P0, CVector3d &P1); // [28] Point-Line Distance
    CVector3d Max(CVector3d &obj);
    CVector3d Min(CVector3d &obj);
    CVector3d RotateX(double rad);                          // [31] Rotate rad around the X axis
    CVector3d RotateY(double rad);                          // [32] Rotate rad around the Y axis
    CVector3d RotateZ(double rad);                          // [33] Rotate rad around the Z axis

    CVector3d NormalVector(CVector3d b, CVector3d c);       // [34] NormalVector of two vectors

    /***** Compatible with CVector3d *****/
};

}