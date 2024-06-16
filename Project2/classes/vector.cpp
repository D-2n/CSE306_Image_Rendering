#include "vector.h"
#include <cmath>
/*
Vector::Vector(double x, double y, double w) {
    coords[0] = x;
    coords[1] = y;
    coords[2] = 0;
    weight = w;
}
*/
Vector::Vector(double x, double y,double z) {
    coords[0] = x;
    coords[1] = y;
    coords[2] = z;
}

WeightedVector::WeightedVector(double x, double y, double z, double w) : Vector(x, y, z), weight(w){};

WeightedVector::WeightedVector(Vector &v, double w) : Vector(v), weight(w){};


double Vector::dot(const Vector &vector) const {
    return coords[0] *  vector[0] + coords[1] *  vector[1] + coords[2] *  vector[2];
}
double Vector::norm2() const {
    return coords[0] * coords[0] + coords[1] * coords[1] + coords[2] * coords[2];

}
double Vector::norm() const {
    return sqrt(norm2());
}

Vector Vector::cross(const Vector &vector) const {
    return Vector(coords[1] * vector[2] - coords[2] * vector[1],
                  coords[2] * vector[0] - coords[0] * vector[2],
                  coords[0] * vector[1] - coords[1] * vector[0]);
}
//Operators ---------------------------------------------------------------------------------------
Vector Vector::operator-(const Vector &vector) const {
    return Vector(coords[0] - vector.coords[0], coords[1] - vector.coords[1], coords[2] - vector.coords[2]);
}
Vector Vector::operator+(const Vector &vector) const {
    return Vector(coords[0] + vector.coords[0], coords[1] + vector.coords[1], coords[2] + vector.coords[2]);
}
Vector Vector::operator*(const double &num) const {
    return Vector(coords[0] * num, coords[1] * num, coords[2] * num);
}
Vector Vector::operator/(const double &num) const {
    return Vector(coords[0] / num, coords[1] / num, coords[2] / num);
}
bool Vector::operator==(const Vector &vector) const {
    return (coords[0] == vector.coords[0]) && (coords[1] == vector.coords[1]) &&  (coords[2] == vector.coords[2]);
}
//save_svg didnt work without the following two
double Vector::operator[](int i) const { 
     return coords[i]; 
}

double &Vector::operator[](int i) {
     return coords[i]; 
}