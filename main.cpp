#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cmath>
#include <limits>
#include <iostream>

struct Vec3 {
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator + (const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator - (const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator * (double d) const { return Vec3(x * d, y * d, z * d); }
    Vec3 operator / (double d) const { return Vec3(x / d, y / d, z / d); }

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

    Sphere(const Vec3& center, double radius) : center(center), radius(radius) {}

    bool intersect(const Ray& ray, double& t) const {
        Vec3 oc = ray.origin - center;
        double b = 2 * oc.dot(ray.direction);
        double c = oc.dot(oc) - radius * radius;
        double discriminant = b * b - 4 * c;
        if (discriminant < 0) return false;
        else {
            discriminant = std::sqrt(discriminant);
            t = -b - discriminant / 2;
            if (t < 0) t = -b + discriminant / 2;
            return t >= 0;
        }
    }
};

Vec3 castRay(const Ray& ray, const Sphere& sphere) {
    double t = std::numeric_limits<double>::max();
    if (!sphere.intersect(ray, t)) return Vec3(0, 0, 0);
    Vec3 point = ray.origin + ray.direction * t;
    Vec3 normal = (point - sphere.center).normalize();
    return Vec3(1, 1, 1);
}

void saveImage(const std::string& filename, const unsigned char* data, int width, int height) {
    stbi_write_png(filename.c_str(), width, height, 3, data, width * 3);
}

int main() {
    int width = 640, height = 480;
    Vec3 origin(0, 0, 0);
    Sphere sphere(Vec3(0, 0, -1), 0.5);
    unsigned char* image = new unsigned char[width * height * 3];

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            double u = (double)x / (double)width;
            double v = (double)y / (double)height;
            Vec3 direction = Vec3(2 * u - 1, 2 * v - 1, -1);
            direction.normalize();
            Vec3 color = castRay(Ray(origin, direction), sphere);

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

