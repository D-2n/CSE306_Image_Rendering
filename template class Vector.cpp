#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <iostream>
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
Vector random_cos() {
    double r1 = ((double)rand() / (double)RAND_MAX); //from https://stackoverflow.com/questions/1340729/how-do-you-generate-a-random-double-uniformly-distributed-between-0-and-1-from-c
    double r2 = ((double)rand() / (double)RAND_MAX);
    double x = std::cos(2 * M_PI * r1) * sqrt(1 - r2);
    double y = std::sin(2 * M_PI * r1) * sqrt(1 - r2);
    double z = sqrt(r2);
    return Vector(x, y, z);

}
class Ray {
public:
    Ray(Vector origin, Vector unit_direction) {
        ray_origin = origin;
        ray_unit_direction = unit_direction;
    };
    Vector ray_origin;
    Vector ray_unit_direction;
};
class Sphere {
public:
    Sphere(Vector center, double radius, Vector albedo, int mirror_status, int refract_status) {
        sphere_center = center;
        sphere_radius = radius;
        sphere_albedo = albedo;
        mirror = mirror_status;
        refrac = refract_status;
    };
    Sphere() {

    };

    Vector sphere_center;
    double sphere_radius;
    Vector sphere_albedo;
    bool mirror;
    bool refrac;
    bool inside = false;

};
class Intersection {
public:
    Intersection(double t_inter, Sphere S_inter, bool does_intersec) {
        t = t_inter;
        S = S_inter;
        intersec = does_intersec;
    };

    Sphere S;
    double t;
    bool intersec;
};
class Scene {
public:
    Scene(Sphere wall_1, Sphere wall_2, Sphere wall_3, Sphere wall_4, Sphere wall_5, Sphere wall_6, Sphere S, Sphere S_1, Sphere S_2, Sphere S_3) {
        objects.push_back(wall_1);
        objects.push_back(wall_2);
        objects.push_back(wall_3);
        objects.push_back(wall_4);
        objects.push_back(wall_5);
        objects.push_back(wall_6);
        objects.push_back(S);
        objects.push_back(S_1);
        objects.push_back(S_2);
        objects.push_back(S_3);
    }
    Intersection scene_intersect(Ray& ray, Vector& color_wall, Vector& P, Vector& N) const {
        Vector u = ray.ray_unit_direction;
        Vector O = ray.ray_origin;
        double t_min = 1e100;
        Sphere S_intersec = objects[0];
        for (Sphere ball : objects) {

            Vector C = ball.sphere_center;
            double t;
            Vector O_minus_C = O - C;
            double discriminant = dot(u, O_minus_C) * dot(u, O_minus_C) - O_minus_C.norm2() + (ball.sphere_radius) * (ball.sphere_radius);
            if (discriminant >= 0) {
                double t1 = dot(u, C - O) - sqrt(discriminant);
                double t2 = dot(u, C - O) + sqrt(discriminant);
                if (t2 >= 0) {
                    if (t1 >= 0) {
                        t = t1;
                        if (t < t_min) {
                            P = O + t * u;
                            N = (P - C) / (P - C).norm();
                            t_min = t;
                            S_intersec = ball;
                        }
                    }
                    else {
                        t = t2;
                        if (t < t_min) {
                            P = O + t * u;
                            N = (P - C) / (P - C).norm();
                            t_min = t;
                            S_intersec = ball;
                        }
                    }
                }
            }
        }
        return Intersection(t_min, S_intersec, true);
    };
    std::vector<Sphere> objects;



    Vector getColor(Ray& ray, int ray_depth, Sphere& sphere, Vector& P, Vector& N, Vector& w_i) {

        Vector color;
        Vector color_wall;
        Vector source(-10, 20, 40);
        if (ray_depth < 0) {
            return Vector(255., 0., 0.);
        }
        if (sphere.mirror) {
            Vector w_r = w_i - (2 * dot(w_i, N) * N);
            w_r.normalize();
            Ray reflected_ray(P + 1e-8 * N, w_r);
            Intersection intersec = scene_intersect(reflected_ray, color, P, N);
            return getColor(reflected_ray, ray_depth - 1, intersec.S, P, N, w_r);
        }
        else {
            if (sphere.refrac) {
                N = (sphere.inside) ? -1 * N : N;
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
                    Intersection intersec = scene_intersect(reflected_ray, color, P, N);
                    return getColor(reflected_ray, ray_depth - 1, intersec.S, P, N, w_r);
                }
                else {
                    double k0 = (std::pow(n1 - n2, 2)) / (std::pow(n1 + n2, 2));
                    double R = k0 + (1 - k0) * std::pow(1 - abs(dot(N, w_i)), 5);
                    double fresnel_refrac = ((double)rand() / (double)RAND_MAX);
                    if (fresnel_refrac < R) {
                        Vector w_r = w_i - (2 * dot(w_i, N) * N);
                        w_r.normalize();
                        Ray reflected_ray(P + 0.0001 * N, w_r);
                        Intersection intersec = scene_intersect(reflected_ray, color, P, N);
                        return getColor(reflected_ray, ray_depth - 1, intersec.S, P, N, w_r);
                    }
                    else {
                        N = N1;
                        Vector w_t_T = (n1 / n2) * (w_i - (dot(w_i, N) * N));
                        Vector w_t_N = -1 * N * sqrt(root);
                        Vector w_t = w_t_T + w_t_N;
                        //w_t.normalize();

                        Ray ray_ref(P - 0.001 * N, w_t);

                        Intersection intersec = scene_intersect(ray_ref, color, P, N);
                        return getColor(ray_ref, ray_depth - 1, intersec.S, P, N, w_t);
                    }
                }
            }

        }
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

        Intersection intersec_shadow = scene_intersect(shadow_ray, color_shadow, P_shadow, N_shadow);

        if (((P_shadow - P).norm2() <= (source - P).norm2())) {
            VpS = 0;
        }
        Vector light = sphere.sphere_albedo;
        light.normalize();
        light =
            intensity / (4 * M_PI * d * d) * (light / M_PI) * VpS * Nw_i;

        final_color = light;
        Vector ray_dir = random_cos();
        double x, y;
        //boxMuller(0.5, x, y);
        //ray_dir.data[0] += x;
        //ray_dir.data[1] += y;
        Ray randomRay(P + eps * N, ray_dir);
        Vector color_random;
        Vector P_rand, N_rand;
        Intersection intersec_random = scene_intersect(randomRay, color_random, P_rand, N_rand);
        Vector mult_color = intersec_random.S.sphere_albedo;
        // ^^ doesnt want to do get color for some reason

        final_color.data[0] += sphere.sphere_albedo.data[0] * mult_color.data[0];
        final_color.data[1] += sphere.sphere_albedo.data[1] * mult_color.data[1];
        final_color.data[2] += sphere.sphere_albedo.data[2] * mult_color.data[2];
        //final_color.normalize();
        return final_color;
    };
};
int main() {
    int W = 512;
    int H = 512;
    Vector Q(0, 0, 55);
    double f = 60.0;
    double alpha = 60.0;

    Sphere S(Vector(-20, 0, 0), 10, Vector(255, 67, 189), true, false);

    Sphere S_1(Vector(0, 0, 0), 10, Vector(255, 67, 189), false, true);

    Sphere S_2(Vector(20, 0, 0), 10, Vector(255, 67, 189), false, true);
    Sphere S_3(Vector(20, 0, 0), 9.5, Vector(255, 67, 189), false, true);
    S_3.inside = true;
    Sphere wall_1(Vector(0, 1000, 0), 940, Vector(255, 0, 255), false, false); // red wall
    Sphere wall_2(Vector(0, 0, 1000), 940, Vector(0, 255, 0), false, false); // green wall
    Sphere wall_5(Vector(1000, 0, 0), 940, Vector(0, 255, 255), false, false); //side left
    Sphere wall_6(Vector(-1000, 0, 0), 940, Vector(255, 0, 0), false, false); //side right
    Sphere wall_3(Vector(0, -1000, 0), 990, Vector(0, 0, 255), false, false); // blue wall
    Sphere wall_4(Vector(0, 0, -1000), 940, Vector(255, 255, 0), false, false); // yellow wall

    Scene main_scene(wall_1, wall_2, wall_3, wall_5, wall_6, wall_4, S, S_1, S_2, S_3);
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
            int n_rays = 1000;
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
                Intersection intersec_wall = main_scene.scene_intersect(ray_camera, color_wall, P, N);
                color_inter = main_scene.getColor(ray_camera, 5, intersec_wall.S, P, N, ray_direction);
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
