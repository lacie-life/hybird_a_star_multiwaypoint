#define _USE_MATH_DEFINES

#include "global/cvector_3d.hpp"
#include <iostream>
#include <cmath>

using namespace std;

namespace PathfindingForVehicles::ReedsSheppPaths
{
//########################################//
// member function of CVector3d, friend function //
//########################################//
/***** Constructor & Destructor *****/

//[01] Constructor
CVector3d::CVector3d()
{
    x = y = z = 0.0;
}

//【02】Constructor
CVector3d::CVector3d(double X, double Y, double Z)
{
    x = X;
    y = Y;
    z = Z;
}

//[03] Destructor
CVector3d::~CVector3d() {}

/**************** Element relation ****************/

//[04] Input coordinate value
void CVector3d::Set(double X, double Y, double Z)
{
    x = X;
    y = Y;
    z = Z;
}

//[05] Normalize after setting the coordinates
void CVector3d::UnitVector(double X, double Y, double Z)
{
    double norm = std::sqrt(X * X + Y * Y + Z * Z);
    x = X / norm;
    y = Y / norm;
    z = Z / norm;
}

//[06] Clear vector
void CVector3d::Clear()
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

//[07] Print output
void CVector3d::Print()
{
    std::cout << "(" << x << "," << y << "," << z << ")" << std::endl;
}

/*************** Overload ***************/

//[10] = operator overload
CVector3d &CVector3d::operator=(const CVector3d &obj)
{
    x = obj.x;
    y = obj.y;
    z = obj.z;
    return *this;
}

//[11] + operator overload
CVector3d CVector3d::operator+(const CVector3d &obj)
{
    CVector3d tmp;
    tmp.x = x + obj.x;
    tmp.y = y + obj.y;
    tmp.z = z + obj.z;
    return (tmp);
}

//[12] += operator overload
CVector3d &CVector3d::operator+=(const CVector3d &obj)
{
    x += obj.x;
    y += obj.y;
    z += obj.z;
    return *this;
}

//【13】- operator overload
CVector3d CVector3d::operator-(const CVector3d &obj)
{
    CVector3d tmp;
    tmp.x = x - obj.x;
    tmp.y = y - obj.y;
    tmp.z = z - obj.z;
    return (tmp);
}

//[14] -= operator overload
CVector3d &CVector3d::operator-=(const CVector3d &obj)
{
    x -= obj.x;
    y -= obj.y;
    z -= obj.z;
    return *this;
}

//【15】-Operator overload Reverse the sign
CVector3d CVector3d::operator-()
{
    CVector3d tmp;
    tmp.x = -x;
    tmp.y = -y;
    tmp.z = -z;
    return (tmp);
}

//[16] * operator overload
CVector3d CVector3d::operator*(const double k)
{
    CVector3d tmp;
    tmp.x = k * x;
    tmp.y = k * y;
    tmp.z = k * z;
    return (tmp);
}
//[17] * operator overload
CVector3d operator*(const double k, const CVector3d &obj)
{
    CVector3d tmp;
    tmp.x = k * obj.x;
    tmp.y = k * obj.y;
    tmp.z = k * obj.z;
    return (tmp);
}

//[19]/ operator overloading
CVector3d CVector3d::operator/(const double k)
{
    CVector3d tmp;
    tmp.x = x / k;
    tmp.y = y / k;
    tmp.z = z / k;
    return (tmp);
}

//【20】* operator overload
CVector3d::operator double *()
{
    static double r[3];
    r[0] = x;
    r[1] = y;
    r[2] = z;
    return r;
}

/**************** Vector calculation ****************/

// [20] Find the midpoint of the vector
CVector3d CVector3d::Mid(CVector3d &obj)
{
    CVector3d tmp;
    tmp.x = (x + obj.x) / 2.0;
    tmp.y = (y + obj.y) / 2.0;
    tmp.z = (z + obj.z) / 2.0;
    return (tmp);
}


//[21] Normalize the stored coordinates
CVector3d CVector3d::Normalize()
{
    double norm = Norm();
    CVector3d tmp;
    tmp.x = x / norm;
    tmp.y = y / norm;
    tmp.z = z / norm;
    return (tmp);
}

//[22] Normalize
void CVector3d::Normalize(CVector3d &obj)
{
    double norm = obj.Norm();
    x = obj.x / norm;
    y = obj.y / norm;
    z = obj.z / norm;
}

//[23] Euclidean norm
double CVector3d::Norm()
{
    return (std::sqrt(x * x + y * y + z * z));
}

//[24] Distance calculation
double CVector3d::Distance(CVector3d &obj)
{
    double X = x - obj.x;
    double Y = y - obj.y;
    double Z = z - obj.z;
    return (std::sqrt(X * X + Y * Y + Z * Z));
}

//[24] Distance calculation
double CVector3d::SquaredDistance(CVector3d &obj)
{
    double X = x - obj.x;
    double Y = y - obj.y;
    double Z = z - obj.z;
    return (X * X + Y * Y + Z * Z);
}

//[25] Inner product of vectors
double CVector3d::Dot(CVector3d &obj)
{
    double dot = x * obj.x;
    dot += y * obj.y;
    dot += z * obj.z;
    return (dot);
}

//[26] Cross product of vectors
CVector3d CVector3d::Cross(CVector3d &obj)
{
    CVector3d tmp;
    tmp.x = y * obj.z - z * obj.y;
    tmp.y = z * obj.x - x * obj.z;
    tmp.z = x * obj.y - y * obj.x;
    return (tmp);
}

//[27] Angle with a
double CVector3d::Angle(CVector3d &obj)
{
    double deg, den;
    den = Norm() * obj.Norm();
    if (den < 1.0e-15)
    {
        std::cout << "error" << std::endl;
        return 0;
    }
    deg = Dot(obj) / den;
    if (deg > 1.0)
        deg = 1.0;
    if (deg < -1.0)
        deg = -1.0;

    deg = std::acos(deg);
    deg = 180.0 * (deg / M_PI);

    return deg;
}

//【28】Point-Line Distance
double CVector3d::PointLineDistance(CVector3d &P0, CVector3d &P1)
{
    CVector3d A = P1 - P0;
    CVector3d B(x - P0.x, y - P0.y, z - P0.z);
    return ((A.Cross(B)).Norm() / P0.Distance(P1));
}

//[29] Return xyz maximum value with point a
CVector3d CVector3d::Max(CVector3d &obj)
{
    CVector3d tmp;
    tmp.x = max(x, obj.x);
    tmp.y = max(y, obj.y);
    tmp.z = max(z, obj.z);
    return (tmp);
}


//[30] Return xyz each minimum value with point a
CVector3d CVector3d::Min(CVector3d &obj)
{
    CVector3d tmp;
    tmp.x = min(x, obj.x);
    tmp.y = min(y, obj.y);
    tmp.z = min(z, obj.z);
    return (tmp);
}


//[31] Rad rotation around X axis
//CVector3d CVector3d::RotateX(double rad)
//{
//    CVector3d tmp;
//    tmp.x = x;
//    tmp.y = (y * std::cos(rad)) + (z * std::sin(rad));
//    tmp.z = -(y * std::sin(rad)) + (z * std::cos(rad));
//    return (tmp);
//}

//[32] rad rotation around Y axis
//CVector3d CVector3d::RotateY(double rad)
//{
//    CVector3d tmp;
//    tmp.x = (x * std::cos(rad)) + (z * std::sin(rad));
//    tmp.y = - (x * std::sin(rad)) + (y * std::cos(rad));
//    tmp.z = 1; //(z * std::cos(-rad)) + (x * std::sin(-rad));
//    return (tmp);
//}

//[33] rad rotation around Z axis
//CVector3d CVector3d::RotateZ(double rad)
//{
//    CVector3d tmp;
//    tmp.x = (x * std::cos(rad)) - (y * std::sin(rad));
//    tmp.y = y; //- (x * std::sin(-rad)) + (y * std::cos(-rad));
//    tmp.z = (x * std::sin(rad)) + (z * std::cos(rad));
//    return (tmp);
//}


//[31] Rad rotation around X axis
CVector3d CVector3d::RotateX(double rad)
{
    CVector3d tmp;
    tmp.x = -x;
    tmp.y = (y * std::cos(-rad)) + (z * std::sin(-rad));
    tmp.z = -(y * std::sin(-rad)) + (z * std::cos(-rad));
    return (tmp);
}

//[32] rad rotation around Y axis
CVector3d CVector3d::RotateY(double rad)
{
    CVector3d tmp;
    tmp.x = -(z * std::sin(-rad)) + (x * std::cos(-rad));
    tmp.y = y;
    tmp.z = (z * std::cos(-rad)) + (x * std::sin(-rad));
    return (tmp);
}

//[33] rad rotation around Z axis
CVector3d CVector3d::RotateZ(double rad)
{
    CVector3d tmp;
    tmp.x = (x * std::cos(-rad)) + (y * std::sin(-rad));
    tmp.y = -(x * std::sin(-rad)) + (y * std::cos(-rad));
    tmp.z = z;
    return (tmp);
}

//[31] Rad rotation around X axis
//CVector3d CVector3d::RotateX(double rad)
//{
//    CVector3d tmp;
//    tmp.x = x;
//    tmp.y = (y * std::cos(rad)) - (z * std::sin(rad));
//    tmp.z = (y * std::sin(rad)) + (z * std::cos(rad));
//    return (tmp);
//}

//[32] rad rotation around Z axis
//CVector3d CVector3d::RotateY(double rad)
//{
//    CVector3d tmp;
//    tmp.x = (z * std::sin(rad)) + (x * std::cos(rad));
//    tmp.y = y;
//    tmp.z = (z * std::cos(rad)) - (x * std::sin(rad));
//    return (tmp);
//}

//[33] rad rotation around Y axis
// CVector3d CVector3d::RotateZ(double rad)
// {
//    CVector3d tmp;
//    tmp.x = (x * std::cos(rad)) - (y * std::sin(rad));
//    tmp.y = (x * std::sin(rad)) + (y * std::cos(rad));
//    tmp.z = z;
//    return (tmp);
// }


//【34】Return the unit normal vector of 3 points a,b,c
CVector3d CVector3d::NormalVector(CVector3d b, CVector3d c)
{
    CVector3d tmp;
    b.x = b.x - x;
    b.y = b.y - y;
    b.z = b.z - z;

    c.x = c.x - x;
    c.y = c.y - y;
    c.z = c.z - z;

    tmp = (0.5 * b.Cross(c)).Normalize();
    return (tmp);
}

/**************** Compatible with CPoint3d ****************/
}
