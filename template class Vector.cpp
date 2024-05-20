#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <iostream>

#include <string>
#include <stdio.h>
#include <algorithm>
#include <vector>

#include <random>
#include <math.h>
#include <cmath>
# define M_PI           3.14159265358979323846  /* pi */
class Vector {
public:
    explicit Vector(double x = 0, double y = 0, double z = 0) {
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }
    double norm2() const {
        return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
    }
    double norm() const {
        return sqrt(norm2());
    }
    void normalize() {
        double n = norm();
        data[0] /= n;
        data[1] /= n;
        data[2] /= n;
    }
    double operator[](int i) const { return data[i]; };
    double& operator[](int i) { return data[i]; };
    double data[3];
};

Vector operator+(const Vector& a, const Vector& b) {
    return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
}
bool operator==(const Vector& a, const Vector& b) {
    return (a[0] == b[0] && a[1] == b[1] && a[2] == b[2]);
}
Vector operator-(const Vector& a, const Vector& b) {
    return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}
Vector operator*(const double a, const Vector& b) {
    return Vector(a * b[0], a * b[1], a * b[2]);
}
Vector operator*(const Vector& a, const double b) {
    return Vector(a[0] * b, a[1] * b, a[2] * b);
}
Vector operator/(const Vector& a, const double b) {
    return Vector(a[0] / b, a[1] / b, a[2] / b);
}
double dot(const Vector& a, const Vector& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
Vector cross(const Vector& a, const Vector& b) {
    return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
}
//from lecture notes
static std::default_random_engine engine(10); // random s e e d = 10
static std::uniform_real_distribution<double> uniform(0, 1);

void boxMuller(double stdev, double& x, double& y) {
    double r1 = uniform(engine);
    double r2 = uniform(engine);
    x = sqrt(-2 * log(r1)) * cos(2 * M_PI * r2) * stdev;
    y = sqrt(-2 * log(r1)) * sin(2 * M_PI * r2) * stdev;
}
Vector random_cos(const Vector& N) {
    double r1 = ((double)rand() / (double)RAND_MAX); //from https://stackoverflow.com/questions/1340729/how-do-you-generate-a-random-double-uniformly-distributed-between-0-and-1-from-c
    double r2 = ((double)rand() / (double)RAND_MAX);
    double x = std::cos(2 * M_PI * r1) * sqrt(1 - r2);
    double y = std::sin(2 * M_PI * r1) * sqrt(1 - r2);
    double z = sqrt(r2);
    Vector T1, T2;
    if (abs(N.data[0]) < abs(N.data[1]) && abs(N.data[0]) < abs(N.data[2])) {
        T1 = Vector(0, -1 * N.data[2], N.data[1]);
    }

    if (abs(N.data[1]) < abs(N.data[0]) && abs(N.data[1]) < abs(N.data[2])) {
        T1 = Vector(-1 * N.data[2], 0, N.data[0]);
    }
    else {
        T1 = Vector(-1 * N.data[1], N.data[0], 0);
    }
    T1.normalize();
    T2 = cross(N, T1);
    Vector result = x * T1 + y * T2 + z * N;
    return result;

}
class Geometry;
class Intersection {
public:
    Intersection(double t_inter, const Geometry* S_inter, bool does_intersec) {
        t = t_inter;
        S = S_inter;
        intersec = does_intersec;
    };
    Intersection() : t(0), S(nullptr), intersec(false) {};

    const Geometry* S;
    double t;
    bool intersec;

};

class Ray {
public:
    Ray(Vector origin, Vector unit_direction) {
        ray_origin = origin;
        ray_unit_direction = unit_direction;
    };
    Vector ray_origin;
    Vector ray_unit_direction;
};
class Geometry {
public:
    Vector geometry_albedo;
    bool mirror;
    bool refrac;
    Vector sphere_center;
    bool inside; // single-case use, so manual trigger
    Geometry(const Vector& albedo, bool mirror, bool refrac, const Vector& center) : geometry_albedo(albedo), mirror(mirror), refrac(refrac), sphere_center(center) {};
    Geometry() : geometry_albedo(Vector(0., 0., 0.)), mirror(false), refrac(false), sphere_center(Vector(0., 0., 0.)) {};

    virtual Intersection intersect(const Ray& ray) const = 0;

    virtual ~Geometry() {}
};

class Sphere : public Geometry {
public:
    Sphere(const Vector& center, double radius, const Vector& albedo, bool mirror_status, bool refract_status)
        : sphere_radius(radius), Geometry(albedo, mirror_status, refract_status, center) {}
    Sphere() : Geometry(Vector(1, 1, 1), false, false, Vector(0, 0, 0)), sphere_radius(1.0) {
        // Initializes a default sphere with radius 1 and white albedo at origin
    }
    Intersection intersect(const Ray& ray) const override {

        Vector u = ray.ray_unit_direction;
        Vector O = ray.ray_origin;
        double t = 1e100;
        Vector C = sphere_center;
        Vector O_minus_C = O - C;
        double discriminant = dot(u, O_minus_C) * dot(u, O_minus_C) - O_minus_C.norm2() + (sphere_radius) * (sphere_radius);
        if (discriminant >= 0) {
            double t1 = dot(u, C - O) - sqrt(discriminant);
            double t2 = dot(u, C - O) + sqrt(discriminant);
            if (t2 >= 0) {
                if (t1 >= 0) {
                    t = t1;
                }
                else {
                    t = t2;
                }
            }

        }
        return Intersection(t, this, true);
    }
    double sphere_radius;

};
class TriangleIndices {
public:
    TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
    };
    int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
    int uvi, uvj, uvk;  // indices within the uv coordinates array
    int ni, nj, nk;  // indices within the normals array
    int group;       // face group
};

class TriangleMesh {
public:
    ~TriangleMesh() {}
    TriangleMesh() {};

    void readOBJ(const char* obj) {

        char matfile[255];
        char grp[255];

        FILE* f;
        f = fopen(obj, "r");
        int curGroup = -1;
        while (!feof(f)) {
            char line[255];
            if (!fgets(line, 255, f)) break;

            std::string linetrim(line);
            linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
            strcpy(line, linetrim.c_str());

            if (line[0] == 'u' && line[1] == 's') {
                sscanf(line, "usemtl %[^\n]\n", grp);
                curGroup++;
            }

            if (line[0] == 'v' && line[1] == ' ') {
                Vector vec;

                Vector col;
                if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
                    col[0] = std::min(1., std::max(0., col[0]));
                    col[1] = std::min(1., std::max(0., col[1]));
                    col[2] = std::min(1., std::max(0., col[2]));

                    vertices.push_back(vec);
                    vertexcolors.push_back(col);

                }
                else {
                    sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                    vertices.push_back(vec);
                }
            }
            if (line[0] == 'v' && line[1] == 'n') {
                Vector vec;
                sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                normals.push_back(vec);
            }
            if (line[0] == 'v' && line[1] == 't') {
                Vector vec;
                sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
                uvs.push_back(vec);
            }
            if (line[0] == 'f') {
                TriangleIndices t;
                int i0, i1, i2, i3;
                int j0, j1, j2, j3;
                int k0, k1, k2, k3;
                int nn;
                t.group = curGroup;

                char* consumedline = line + 1;
                int offset;

                nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
                if (nn == 9) {
                    if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                    if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                    if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                    if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                    if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                    if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                    if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                    if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                    if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                    indices.push_back(t);
                }
                else {
                    nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
                    if (nn == 6) {
                        if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                        if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                        if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                        if (j0 < 0) t.uvi = uvs.size() + j0; else   t.uvi = j0 - 1;
                        if (j1 < 0) t.uvj = uvs.size() + j1; else   t.uvj = j1 - 1;
                        if (j2 < 0) t.uvk = uvs.size() + j2; else   t.uvk = j2 - 1;
                        indices.push_back(t);
                    }
                    else {
                        nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
                        if (nn == 3) {
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                            indices.push_back(t);
                        }
                        else {
                            nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
                            if (i0 < 0) t.vtxi = vertices.size() + i0; else t.vtxi = i0 - 1;
                            if (i1 < 0) t.vtxj = vertices.size() + i1; else t.vtxj = i1 - 1;
                            if (i2 < 0) t.vtxk = vertices.size() + i2; else t.vtxk = i2 - 1;
                            if (k0 < 0) t.ni = normals.size() + k0; else    t.ni = k0 - 1;
                            if (k1 < 0) t.nj = normals.size() + k1; else    t.nj = k1 - 1;
                            if (k2 < 0) t.nk = normals.size() + k2; else    t.nk = k2 - 1;
                            indices.push_back(t);
                        }
                    }
                }

                consumedline = consumedline + offset;

                while (true) {
                    if (consumedline[0] == '\n') break;
                    if (consumedline[0] == '\0') break;
                    nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
                    TriangleIndices t2;
                    t2.group = curGroup;
                    if (nn == 3) {
                        if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                        if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                        if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                        if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                        if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                        if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                        if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                        if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                        if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;
                        indices.push_back(t2);
                        consumedline = consumedline + offset;
                        i2 = i3;
                        j2 = j3;
                        k2 = k3;
                    }
                    else {
                        nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
                        if (nn == 2) {
                            if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                            if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                            if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                            if (j0 < 0) t2.uvi = uvs.size() + j0; else  t2.uvi = j0 - 1;
                            if (j2 < 0) t2.uvj = uvs.size() + j2; else  t2.uvj = j2 - 1;
                            if (j3 < 0) t2.uvk = uvs.size() + j3; else  t2.uvk = j3 - 1;
                            consumedline = consumedline + offset;
                            i2 = i3;
                            j2 = j3;
                            indices.push_back(t2);
                        }
                        else {
                            nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
                            if (nn == 2) {
                                if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                if (k0 < 0) t2.ni = normals.size() + k0; else   t2.ni = k0 - 1;
                                if (k2 < 0) t2.nj = normals.size() + k2; else   t2.nj = k2 - 1;
                                if (k3 < 0) t2.nk = normals.size() + k3; else   t2.nk = k3 - 1;
                                consumedline = consumedline + offset;
                                i2 = i3;
                                k2 = k3;
                                indices.push_back(t2);
                            }
                            else {
                                nn = sscanf(consumedline, "%u%n", &i3, &offset);
                                if (nn == 1) {
                                    if (i0 < 0) t2.vtxi = vertices.size() + i0; else    t2.vtxi = i0 - 1;
                                    if (i2 < 0) t2.vtxj = vertices.size() + i2; else    t2.vtxj = i2 - 1;
                                    if (i3 < 0) t2.vtxk = vertices.size() + i3; else    t2.vtxk = i3 - 1;
                                    consumedline = consumedline + offset;
                                    i2 = i3;
                                    indices.push_back(t2);
                                }
                                else {
                                    consumedline = consumedline + 1;
                                }
                            }
                        }
                    }
                }

            }

        }
        fclose(f);

    }

    double intersect(Ray& ray, Vector& N) {
        Vector O = ray.ray_origin;
        Vector u = ray.ray_unit_direction;
        double t_min = 10e100;
        double t;
        for (int i = 0; i < vertexcolors.size(); i++) {
            t = dot(vertices[i] - O, N) / dot(u, N);
            if (t < t_min) {
                t_min = t;
            }
        }
        return t_min;
    }
    std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs;
    std::vector<Vector> vertexcolors;

};

class Scene {
public:
    Scene(std::vector<Geometry*> spheres) : objects(spheres) {}

    Intersection scene_intersect(Ray& ray, Vector& P, Vector& N) const {
        double t;
        Vector O = ray.ray_origin;
        Vector u = ray.ray_unit_direction;
        double t_min = 1e100;
        Geometry* S_intersec;
        Vector C;
        Vector N1 = N;
        Intersection inter;
        bool intersects_mesh = false;
        for (auto& ball : objects) {
            Intersection inter1 = ball->intersect(ray);
            t = inter1.t;
            C = ball->sphere_center;
            if (t < t_min) {
                P = O + t * u;
                N = (P - C) / (P - C).norm();
                t_min = t;
                inter.t = t;
                inter.S = ball;
                inter.intersec = true;
            }
        }
        return inter;
    };
    std::vector<Geometry*> objects;
    std::vector<TriangleMesh> mesh_objects; //if i wanna add multiple cats



    Vector getColor(Ray& ray, int ray_depth, Vector& P, Vector& N, Vector& w_i) {

        Vector color;
        Vector source(-10, 20, 40);
        Intersection main_inter = scene_intersect(ray, P, N);
        if (ray_depth < 0 || !main_inter.intersec) {
            return Vector(0., 255., 0.);
        }
        const Geometry* sphere = main_inter.S;
        //------------------ Sphere part
        if (sphere->mirror) {
            Vector w_r = w_i - (2 * dot(w_i, N) * N);
            w_r.normalize();
            Ray reflected_ray(P + 1e-8 * N, w_r);
            return getColor(reflected_ray, ray_depth - 1, P, N, w_r);
        }
        else {
            if (sphere->refrac) {
                N = (sphere->inside) ? -1 * N : N;
                double n1, n2;
                Vector N1;
                n1 = (dot(w_i, N) > 0) ? 1.5 : 1;
                n2 = (dot(w_i, N) > 0) ? 1 : 1.5;
                N1 = (dot(w_i, N) < 0) ? N : -1 * N;
                //if sqrt negative, bounce ray
                double root = 1 - std::pow(n1 / n2, 2) * (1 - std::pow(dot(w_i, N), 2));
                if (root < 0) {
                    Vector w_r = w_i - (2 * dot(w_i, N) * N);
                    w_r.normalize();
                    Ray reflected_ray(P + 0.0001 * N, w_r);
                    return getColor(reflected_ray, ray_depth - 1, P, N, w_r);
                }
                else {
                    double k0 = (std::pow(n1 - n2, 2)) / (std::pow(n1 + n2, 2));
                    double R = k0 + (1 - k0) * std::pow(1 - abs(dot(N, w_i)), 5);
                    double fresnel_refrac = ((double)rand() / (double)RAND_MAX);
                    if (fresnel_refrac < R) {
                        Vector w_r = w_i - (2 * dot(w_i, N) * N);
                        w_r.normalize();
                        Ray reflected_ray(P + 0.0001 * N, w_r);
                        return getColor(reflected_ray, ray_depth - 1, P, N, w_r);
                    }
                    else {
                        N = N1;
                        Vector w_t_T = (n1 / n2) * (w_i - (dot(w_i, N) * N));
                        Vector w_t_N = -1 * N * sqrt(root);
                        Vector w_t = w_t_T + w_t_N;
                        //w_t.normalize();

                        Ray ray_ref(P - 0.001 * N, w_t);

                        return getColor(ray_ref, ray_depth - 1, P, N, w_t);
                    }
                }
            }

        }

        //------------------- Common part
        Vector final_color;

        double eps = 1E-8;
        Vector color_shadow;
        Vector P_shadow, N_shadow;
        double d = (source - P).norm();
        Vector omega_i = (source - P) / d;
        Ray shadow_ray(P + eps * N, omega_i);
        int VpS = 1;


        double Nw_i = std::max(0., dot(N, omega_i));
        double intensity = 2.1e10;

        Intersection intersec_shadow = scene_intersect(shadow_ray, P_shadow, N_shadow);

        if (((P_shadow - P).norm2() <= (source - P).norm2())) {
            VpS = 0;
        }
        Vector light = sphere->geometry_albedo;
        light.normalize();
        light =
            intensity / (4 * M_PI * d * d) * (light / M_PI) * VpS * Nw_i;

        final_color = light;
        Vector ray_dir = random_cos(N);
        Ray randomRay(P + eps * N, ray_dir);
        Vector color_random;
        Vector P_rand, N_rand;

        //same as calling getColor but without the additional lighting operations
        //can modify get color, but i dont want to add more parameters and make it uglier
        Intersection intersec_random = scene_intersect(randomRay, P_rand, N_rand);
        Vector mult_color = intersec_random.S->geometry_albedo;

        final_color.data[0] += sphere->geometry_albedo.data[0] * mult_color.data[0];
        final_color.data[1] += sphere->geometry_albedo.data[1] * mult_color.data[1];
        final_color.data[2] += sphere->geometry_albedo.data[2] * mult_color.data[2];
        return final_color;
    };
};
int main() {
    int W = 512;
    int H = 512;
    Vector Q(0, 0, 55);
    double f = 60.0;
    double alpha = 60.0;

    Sphere* S = new Sphere(Vector(-20, 0, 0), 10, Vector(255, 67, 189), true, false);

    Sphere* S_1 = new Sphere(Vector(0, 0, 0), 10, Vector(255, 67, 189), false, true);

    Sphere* S_2 = new Sphere(Vector(20, 0, 0), 10, Vector(255, 67, 189), false, true);
    Sphere* S_3 = new Sphere(Vector(20, 0, 0), 9.5, Vector(255, 67, 189), false, true);
    S_3->inside = true;
    std::vector<Geometry*> walls = {
        new Sphere(Vector(0, 1000, 0), 940, Vector(255, 0, 255), false, false), // red wall
        new Sphere(Vector(0, 0, 1000), 940, Vector(0, 255, 0), false, false), // green wall
        new Sphere(Vector(1000, 0, 0), 940, Vector(0, 255, 255), false, false), //side left
        new Sphere(Vector(-1000, 0, 0), 940, Vector(255, 0,0), false, false), //side right
        new Sphere(Vector(0, -1000, 0), 990, Vector(0, 0, 255), false, false), // blue wall
        new Sphere(Vector(0, 0, -1000), 940, Vector(255, 255, 0), false, false), // yellow wall
        S,
        S_1,
        S_2,
        S_3
    };
    TriangleIndices triangle_test();
    Scene main_scene(walls);

    Vector light_source(-10, 20, 40); // light source 

    Vector P_shadow, N_shadow, color_shadow;
    std::vector<unsigned char> image(W * H * 3, 0);
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            Vector color_wall(0., 0., 0.);
            Vector color_shadow;

            Vector P, N;
            Vector P_shadow, N_shadow;
            // Camera ray initialization
            Vector pixel_coord(Q.data[0] + j + 0.5 - W / 2, Q.data[1] + H - i - 1 + 0.5 - H / 2, Q.data[2] - W / (2 * tan(alpha * M_PI / 360)));


            double x, y;
            int n_rays = 10;
            double r = 0.;
            double g = 0.;
            double b = 0.;
            for (int counter = 0; counter < n_rays; counter++) {
                Vector ray_direction = pixel_coord - Q;
                boxMuller(0.5, x, y);
                ray_direction.data[0] += x;
                ray_direction.data[1] += y;
                ray_direction.normalize();
                Ray ray_camera(Q, ray_direction);
                Vector color_inter;
                Intersection intersec_wall = main_scene.scene_intersect(ray_camera, P, N);

                color_inter = main_scene.getColor(ray_camera, 5, P, N, ray_direction);
                r += color_inter.data[0];
                g += color_inter.data[1];
                b += color_inter.data[2];
            }
            color_wall.data[0] = r / (double)n_rays;
            color_wall.data[1] = g / (double)n_rays;
            color_wall.data[2] = b / (double)n_rays;

            Vector current_color = color_wall;

            //current_color.normalize();
            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(current_color[0], 1 / 2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(current_color[1], 1 / 2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(current_color[2], 1 / 2.2));

        }
    }
    stbi_write_png("image.png", W, H, 3, &image[0], 0);

    return 0;
}
