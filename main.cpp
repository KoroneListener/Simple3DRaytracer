#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cmath>
#include <limits>
#include <iostream>

struct Vec3 {
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double d) const { return Vec3(x * d, y * d, z * d); }
    Vec3 operator/(double d) const { return Vec3(x / d, y / d, z / d); }

    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }
    Vec3 cross(const Vec3& v) const { return Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }
    Vec3& normalize() { return *this = *this * (1 / std::sqrt(dot(*this))); }
};

struct Ray {
    Vec3 origin, direction;
    Ray(const Vec3& origin, const Vec3& direction) : origin(origin), direction(direction) {}
};

struct Sphere {
    Vec3 center;
    double radius;
    Vec3 color;

    Sphere(const Vec3& center, double radius, const Vec3& color) : center(center), radius(radius), color(color) {}

    bool intersect(const Ray& ray, double& t) const {
        Vec3 oc = ray.origin - center;
        double b = 2 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * c;
        if (discriminant < 0)
            return false;
        else {
            discriminant = std::sqrt(discriminant);
            t = (-b - discriminant) / 2;
            if (t < 0)
                t = (-b + discriminant) / 2;
            return t >= 0;
        }
    }
};

Vec3 castRay(const Ray& ray, const Sphere* spheres, int numSpheres, const Vec3& backgroundColor) {
    double tMin = std::numeric_limits<double>::max();
    const Sphere* hitSphere = nullptr;

    for (int i = 0; i < numSpheres; i++) {
        double t = std::numeric_limits<double>::max();
        if (spheres[i].intersect(ray, t)) {
            if (t < tMin) {
                tMin = t;
                hitSphere = &spheres[i];
            }
        }
    }

    if (hitSphere) {
        Vec3 point = ray.origin + ray.direction * tMin;
        Vec3 normal = (point - hitSphere->center).normalize();
        return hitSphere->color * std::abs(normal.dot(ray.direction));
    }

    return backgroundColor;
}

void saveImage(const std::string& filename, const unsigned char* data, int width, int height) {
    stbi_write_png(filename.c_str(), width, height, 3, data, width * 3);
}

int main() {
    int width = 640, height = 480;
    Vec3 backgroundColor(0.2, 0.7, 0.8);

    Sphere spheres[] = {
        Sphere(Vec3(0, 0, -1), 0.5, Vec3(1, 0, 0)),    // Red sphere
        Sphere(Vec3(0, -100.5, -1), 100, Vec3(0, 0.5, 0))  // Green floor
    };
    int numSpheres = sizeof(spheres) / sizeof(spheres[0]);

    unsigned char* image = new unsigned char[width * height * 3];

    Vec3 origin(0, 0, 0);
    double aspectRatio = (double)width / height;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double u = (x + 0.5) / width;
            double v = (height - y - 0.5) / height;

            double viewportHeight = 2.0;
            double viewportWidth = aspectRatio * viewportHeight;
            double focalLength = 1.0;

            Vec3 direction = Vec3(viewportWidth * (u - 0.5), viewportHeight * (v - 0.5), -focalLength).normalize();

            Vec3 color = castRay(Ray(origin, direction), spheres, numSpheres, backgroundColor);

            int index = (y * width + x) * 3;
            image[index] = static_cast<unsigned char>(color.x * 255);
            image[index + 1] = static_cast<unsigned char>(color.y * 255);
            image[index + 2] = static_cast<unsigned char>(color.z * 255);
        }
    }

    saveImage("output.png", image, width, height);

    delete[] image;
    return 0;
}

