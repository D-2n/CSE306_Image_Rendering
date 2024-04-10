#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <iostream>
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
    /*
    bool intersect(Ray& ray, Vector& color, Vector& P, Vector&N) const {
        Vector u = ray.ray_unit_direction;
        Vector O = ray.ray_origin;
        Vector C = sphere_center;

        Vector ray_color;
        double t;
        Vector O_minus_C = O - C;
        double discriminant = dot(u, O_minus_C) * dot(u, O_minus_C) - O_minus_C.norm2() + sphere_radius * sphere_radius;
        if (discriminant >= 0) {
            double t1 = dot(u, C - O) - sqrt(discriminant);
            double t2 = dot(u, C - O) + sqrt(discriminant);
            if (t2 >= 0) {
                if (t1 >= 0) {
                    t = t1;
                    P = O + t * u;
                    N = (P - C) / (P - C).norm();
                    color = sphere_albedo;
                    return true;
                }
                else {
                    t =  t2;
                    P = O + t * u;
                    N = (P - C) / (P - C).norm();
                    color = sphere_albedo;
                    return true;

                }
            }
        }

        return false;
    };
    */

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
        if (ray_depth < 0) {
            return Vector(255., 0., 0.);
        }
        if (sphere.mirror) {
            Vector w_r = w_i - (2 * dot(w_i, N) * N);
            w_r.normalize();
            Ray reflected_ray(P + 0.0001 * N, w_r);
            Intersection intersec = scene_intersect(reflected_ray, color, P, N);
            return getColor(reflected_ray, ray_depth - 1, intersec.S, P, N, w_r);
        }
        else {
            if (sphere.refrac) {
                if (sphere.inside) {
                    N = -1 * N;
                }
                if (dot(w_i, N) > 0) {
                    double n1 = 1.5;
                    double n2 = 1;
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
                        N = -1 * N;
                        Vector w_t_T = (n1 / n2) * (w_i - (dot(w_i, N) * N));
                        Vector w_t_N = -1 * N * sqrt(root);
                        Vector w_t = w_t_T + w_t_N;
                        //w_t.normalize();

                        Ray ray_ref(P - 0.001 * N, w_t);

                        Intersection intersec = scene_intersect(ray_ref, color, P, N);
                        return getColor(ray_ref, ray_depth - 1, intersec.S, P, N, w_t);
                    }
                }
                else {
                    double n1 = 1;
                    double n2 = 1.5;
                    double root = 1 - std::pow(n1 / n2, 2) * (1 - std::pow(dot(w_i, N), 2));
                    if (root < 0) {
                        Vector w_r = w_i - (2 * dot(w_i, N) * N);
                        w_r.normalize();
                        Ray reflected_ray(P + 0.0001 * N, w_r);
                        Intersection intersec = scene_intersect(reflected_ray, color, P, N);
                        return getColor(reflected_ray, ray_depth - 1, intersec.S, P, N, w_r);
                    }
                    else {
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
            else {
                return sphere.sphere_albedo;
            }
        }
        return Vector(255., 0., 0.);
    };
};
Vector random_cos() {
    double r1 = ((double)rand() / (double)RAND_MAX); //from https://stackoverflow.com/questions/1340729/how-do-you-generate-a-random-double-uniformly-distributed-between-0-and-1-from-c
    double r2 = ((double)rand() / (double)RAND_MAX);
    double x = std::cos(2 * M_PI * r1) * sqrt(1 - r2);
    double y = std::sin(2 * M_PI * r1) * sqrt(1 - r2);
    double z = sqrt(r2);
    return Vector(x, y, z);

}
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
    Sphere wall_1(Vector(0, 1000, 0), 940, Vector(255, 0, 0), false, false); // red wall
    Sphere wall_2(Vector(0, 0, 1000), 940, Vector(0, 255, 0), false, false); // green wall
    Sphere wall_5(Vector(1000, 0, 0), 940, Vector(0, 255, 255), false, false); //side left
    Sphere wall_6(Vector(-1000, 0, 0), 940, Vector(255, 215, 171), false, false); //side right
    Sphere wall_3(Vector(0, -1000, 0), 990, Vector(0, 0, 255), false, false); // blue wall
    Sphere wall_4(Vector(0, 0, -1000), 940, Vector(255, 255, 0), false, false); // yellow wall

    Scene main_scene(wall_1, wall_2, wall_3, wall_5, wall_6, wall_4, S, S_1, S_2, S_3);
    Vector light_source(-10, 20, 40); // light source 

    Vector P_shadow, N_shadow, color_shadow;
    std::vector<unsigned char> image(W * H * 3, 0);
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            Vector color_wall;
            Vector color_shadow;

            Vector P, N;
            Vector P_shadow, N_shadow;
            // Camera ray initialization
            Vector pixel_coord(Q.data[0] + j + 0.5 - W / 2, Q.data[1] + H - i - 1 + 0.5 - H / 2, Q.data[2] - W / (2 * tan(alpha * M_PI / 360)));
            Vector ray_direction = pixel_coord - Q;
            ray_direction.normalize();
            Ray ray_camera(Q, ray_direction);
            int n_rays = 1000;
            Intersection intersec_wall = main_scene.scene_intersect(ray_camera, color_wall, P, N);
            for (int i = 0; i < n_rays; i++) {
                Intersection intersec_wall = main_scene.scene_intersect(ray_camera, color_wall, P, N);
                Vector color_inter = main_scene.getColor(ray_camera, 5, intersec_wall.S, P, N, ray_direction);
                color_wall.data[0] += color_inter.data[0];
                color_wall.data[1] += color_inter.data[1];
                color_wall.data[2] += color_inter.data[2];
            }
            color_wall.data[0] /= n_rays;
            color_wall.data[1] /= n_rays;
            color_wall.data[2] /= n_rays;
            image[(i * W + j) * 3 + 0] = std::max(0., color_wall[0]);
            image[(i * W + j) * 3 + 1] = std::max(0., color_wall[1]);
            image[(i * W + j) * 3 + 2] = std::max(0., color_wall[2]);
            double d = (light_source - P).norm();
            Vector w_i = (light_source - P) / d;
            double epsilon = 1;
            Ray shadow_ray(P + epsilon * N, w_i);
            int VpS = 1;
            Intersection intersec_shadow = main_scene.scene_intersect(shadow_ray, color_shadow, P_shadow, N_shadow);

            if (((P_shadow - P).norm2() <= (light_source - P).norm2())) {
                VpS = 0;
            }

            double Nw_i = std::max(0., dot(N, w_i));
            double intensity = 2.1e10;

            //set the albedo a.k.a the color of the pixel
            Vector current_color(image[(i * W + j) * 3 + 0], image[(i * W + j) * 3 + 1], image[(i * W + j) * 3 + 2]);
            Vector albedo(image[(i * W + j) * 3 + 0], image[(i * W + j) * 3 + 1], image[(i * W + j) * 3 + 2]);

            current_color.normalize();
            // run it through the formula
            current_color = (intensity / (4 * M_PI * d * d)) * (current_color / M_PI) * VpS * Nw_i;
            Vector ray_dir = random_cos();
            Ray randomRay(P, ray_dir);
            Vector mult_color = main_scene.getColor(randomRay, 5, intersec_wall.S, P, N, ray_dir);
            current_color.data[0] += albedo.data[0] * mult_color.data[0];
            current_color.data[1] += albedo.data[1] * mult_color.data[1];
            current_color.data[2] += albedo.data[2] * mult_color.data[2];
            //set pixel color to updated shadow one
            image[(i * W + j) * 3 + 0] = std::min(255., std::pow(current_color[0], 1 / 2.2));
            image[(i * W + j) * 3 + 1] = std::min(255., std::pow(current_color[1], 1 / 2.2));
            image[(i * W + j) * 3 + 2] = std::min(255., std::pow(current_color[2], 1 / 2.2));

        }
    }
    stbi_write_png("image.png", W, H, 3, &image[0], 0);

    return 0;
}
