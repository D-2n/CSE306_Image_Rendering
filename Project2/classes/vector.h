#ifndef VECTOR_H
#define VECTOR_H

class Vector{
public:
    //Vector(double x, double y, double w);
    explicit Vector(double x = 0, double y = 0, double z = 0);
    double coords[3];
    //needed for svg, didnt want to compile
    double operator[](int i) const; 
    double &operator[](int i); 
    //standard operators
    Vector operator+(const Vector &vector) const;
    Vector operator-(const Vector &vector) const;
    bool operator==(const Vector &vector) const;
    Vector operator/(const double &num) const;
    Vector operator*(const double &num) const;
    //dot product
    double dot(const Vector &vector) const;
    double norm2() const;
    Vector cross(const Vector &vector) const;
    
    double norm() const;
};
class WeightedVector : public Vector {
public:
    explicit WeightedVector(double x = 0, double y = 0, double z = 0, double w = 0);
    WeightedVector(Vector &v, double w);
    double weight;
};

#endif