#ifndef POINT3_HPP
#define POINT3_HPP

#include <iostream>
#include <cmath>


class Point3{
    public:
        double e[3];
        Point3() : e{0,0,0} {}
        Point3(double e0, double e1, double e2): e{e0, e1, e2}{}

        double x() const {return e[0];}
        double y() const {return e[1];}
        double z() const {return e[2];}

        double operator[](int i) const {return e[i];}

        Point3& operator*=(const double t){
            e[0] *=t;
            e[1] *=t;
            e[2] *=t;
            return *this;
        }

        Point3& operator/=(const double t){
            return *this *= 1/t;
        }

        double norm() const {
            return std::sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]);
        }

        double norm_sqr() const {
            return e[0]*e[0] + e[1]*e[1] + e[2]*e[2];
        }

};

inline std::ostream& operator<<(std::ostream &out, const Point3& v) {
    return out << "[ " << v.e[0] << " ; " << v.e[1] << " ; " << v.e[2] << " ]";
}

inline Point3 operator+(Point3& u, Point3& v){
            return Point3(u.e[0]+v.e[0], u.e[1]+v.e[1], u.e[2]+v.e[2]);
}

inline Point3 operator+(Point3 u, Point3 v){
            return Point3(u.e[0]+v.e[0], u.e[1]+v.e[1], u.e[2]+v.e[2]);
}

inline Point3 operator-(Point3& u, Point3& v){
    return Point3(u.e[0]-v.e[0], u.e[1]-v.e[1], u.e[2]-v.e[2]);
}

inline Point3 operator*(double& t, Point3 &u){
    return Point3(t*u.e[0], t*u.e[1], t*u.e[2]);
}

inline Point3 operator*(Point3 &u, double& t){
    return Point3(t*u.e[0], t*u.e[1], t*u.e[2]);
}

inline Point3 operator/(Point3& u, double t){
    return Point3((1/t)*u.e[0], (1/t)*u.e[1], (1/t)*u.e[2]);
}


inline double dot(const Point3 &u, const Point3 &v) {
    return u.e[0] * v.e[0]
        + u.e[1] * v.e[1]
        + u.e[2] * v.e[2];
}

inline Point3 cross(const Point3 &u, const Point3 &v) {
    return Point3(u.e[1] * v.e[2] - u.e[2] * v.e[1],
                u.e[2] * v.e[0] - u.e[0] * v.e[2],
                u.e[0] * v.e[1] - u.e[1] * v.e[0]);
}

inline Point3 unit_vector(Point3 v) {
    return v / v.norm();
}

inline double distance(Point3 u, Point3 v){
    return (u-v).norm();
}

using vecteur = Point3;

#endif