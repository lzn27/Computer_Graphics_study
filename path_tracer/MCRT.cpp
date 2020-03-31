// reference
// http://blog.csdn.net/admintan/article/details/71598413

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

struct Vec
{
    double x, y, z;
    Vec(double x_ = 0, double y_ = 0, double z_ = 0) //default=0
    {
        x = x_;
        y = y_;
        z = z_;
    }
    Vec operator=(const Vec &b) const //......marking(=`w`=)
    {
        return Vec(b.x, b.y, b.z);
    }
    Vec operator+(const Vec &b) const
    {
        return Vec(x + b.x, y + b.y, z + b.z);
    }
    Vec operator-(const Vec &b) const
    {
        return Vec(x - b.x, y - b.y, z - b.z);
    }
    Vec operator*(const double b) const
    {
        return Vec(x * b, y * b, z * b);
    }
    Vec mult(const Vec &b) const
    {
        return Vec(x * b.x, y * b.y, z * b.z);
    }
    Vec &norm() //单位化
    {
        return *this = *this * (1 / sqrt(x * x + y * y + z * z));
    }
    double dot(const Vec &b) const //向量点积
    {
        return x * b.x + y * b.y + z * b.z;
    }
    Vec operator%(Vec &b)
    {
        return Vec(y * b.z - z * b.y, z * b.x - x * b.z, x * b.y - y * b.x);
    }
};

struct Ray
{
    Vec o, d; //origin, direction
    Ray(Vec o_, Vec d_)
    {
        o = o_;
        d = d_;
    }
};

enum Refl_t
{
    DIFF,
    SPEC,
    REFR
};

struct Sphere //球体
{
    double rad;
    Vec p, e, c;
    Refl_t refl; //reflection type(diffuse specular,refractive)
    Sphere(double rad_, Vec p_, Vec e_, Vec c_, Refl_t refl_) : rad(rad_), p(p_), e(e_), c(c_), refl(refl_) {}
    double intersect(const Ray &r) const
    {
        Vec op = p - r.o;
        double t, b = op.dot(r.d), det = b * b - (op.dot(op) - rad * rad);
        if (det < 0) //无解
            return 0;
        else
            det = sqrt(det);
        return (t = b - det) > 0.0 ? t : ((t = b + det) > 0.0 ? t : 0);
    }
};

Sphere spheres[] = {
    //Scene: radius, position, emission, color, material
    Sphere(1e5, Vec(1e5 + 1, 40.8, 81.6), Vec(), Vec(.75, .25, .25), DIFF),   //Left
    Sphere(1e5, Vec(-1e5 + 99, 40.8, 81.6), Vec(), Vec(.25, .25, .75), DIFF), //Rght
    Sphere(1e5, Vec(50, 40.8, 1e5), Vec(), Vec(.75, .75, .75), DIFF),         //Back
    Sphere(1e5, Vec(50, 40.8, -1e5 + 170), Vec(), Vec(), DIFF),               //Frnt
    Sphere(1e5, Vec(50, 1e5, 81.6), Vec(), Vec(.75, .75, .75), DIFF),         //Botm
    Sphere(1e5, Vec(50, -1e5 + 81.6, 81.6), Vec(), Vec(.75, .75, .75), DIFF), //Top
    Sphere(16.5, Vec(27, 16.5, 47), Vec(), Vec(1, 1, 1) * .999, SPEC),        //Mirr
    Sphere(16.5, Vec(73, 16.5, 78), Vec(), Vec(1, 1, 1) * .999, REFR),        //Glas
    Sphere(600, Vec(50, 681.6 - .27, 81.6), Vec(12, 12, 12), Vec(), DIFF)     //Lite
};

inline double clamp(double x)
{
    return x < 0 ? 0 : (x > 1 ? 1 : x);
}

inline int toInt(double x)
{
    return int(pow(clamp(x), 1 / 2.2) * 255 + .5);
}

inline bool intersect(const Ray &r, double &t, int &id)
{
    t = 1e20;
    double n = sizeof(spheres) / sizeof(Sphere), d, inf = t;
    for (int i = 0; i < int(n); i++)
    {
        if ((d = spheres[i].intersect(r)) && d < t)
        {
            t = d;
            id = i;
        }
    }
    return t < inf;
}

Vec radiance(const Ray &r, int depth, unsigned short *Xi)
{
    double t; //distance
    int id = 0;
    if (!intersect(r, t, id))        //无交点
        return Vec();                //black
    const Sphere &obj = spheres[id]; //the hit object
    Vec x = r.o + r.d * t, n = (x - obj.p).norm(), nl = n.dot(r.d) < 0 ? n : n * -1, f = obj.c;
    double p = f.x > f.y && f.x > f.z ? f.x : f.y > f.z ? f.y : f.z; //p is max
    if (++depth > 5)
        if (erand48(Xi) < p)
            f = f * (1 / p);
        else
            return obj.e;
    if (obj.refl == DIFF)
    { //漫反射
        double r1 = 2 * M_PI * erand48(Xi), r2 = erand48(Xi), r2s = sqrt(r2);
        Vec w = nl, u = ((fabs(w.x) > .1 ? Vec(0, 1) : Vec(1)) % w).norm(), v = w % u;
        Vec d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).norm();
        return obj.e + f.mult(radiance(Ray(x, d), depth, Xi));
    }
}

int main()
{
}
