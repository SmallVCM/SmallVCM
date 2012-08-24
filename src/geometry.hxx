/*
 * This is published under Apache 2.0
 */

#ifndef __GEOMETRY_HXX__
#define __GEOMETRY_HXX__

#include <vector>
#include <cmath>
#include "math.hxx"
#include "ray.hxx"

//////////////////////////////////////////////////////////////////////////
// Geometry

class AbstractGeometry
{
public:
    virtual ~AbstractGeometry(){};
    // Finds the closest intersection
    virtual bool Intersect (const Ray& aRay, Isect& oResult) const = 0;
    // Finds any intersection, default calls Intersect
    virtual bool IntersectP(const Ray& aRay, Isect& oResult) const
    { return Intersect(aRay, oResult); }

    // Grows given bounding box by this object
    virtual void GrowBBox(Vec3f &aoBBoxMin, Vec3f &aoBBoxMax) = 0;
};

class GeometryList : public AbstractGeometry
{
public:
    virtual ~GeometryList()
    {
        for(int i=0; i<(int)mGeometry.size(); i++)
            delete mGeometry[i];
    };

    virtual bool Intersect(const Ray& aRay, Isect& oResult) const
    {
        bool anyIntersection = false;
        for(int i=0; i<(int)mGeometry.size(); i++)
        {
            bool hit = mGeometry[i]->Intersect(aRay, oResult);
            if(hit)
                anyIntersection = hit;
        }

        return anyIntersection;
    }

    virtual bool IntersectP(const Ray& aRay, Isect& oResult) const
    {
        for(int i=0; i<(int)mGeometry.size(); i++)
        {
            if(mGeometry[i]->IntersectP(aRay, oResult))
                return true;
        }

        return false;
    }

    virtual void GrowBBox(Vec3f &aoBBoxMin, Vec3f &aoBBoxMax)
    {
        for(int i=0; i<(int)mGeometry.size(); i++)
            mGeometry[i]->GrowBBox(aoBBoxMin, aoBBoxMax);
    }
public:
    std::vector<AbstractGeometry*> mGeometry;
};

class Triangle : public AbstractGeometry
{
public:
    Triangle(){}
    Triangle(const Vec3f& p0, const Vec3f& p1, const Vec3f& p2, int aMatID)
    {
        p[0] = p0; p[1] = p1; p[2] = p2; matID = aMatID;
    }

    virtual bool Intersect(const Ray& aRay, Isect& oResult) const
    {
        const Vec3f ao = p[0] - aRay.org;
        const Vec3f bo = p[1] - aRay.org;
        const Vec3f co = p[2] - aRay.org;

        const Vec3f v0 = Cross(co, bo);
        const Vec3f v1 = Cross(bo, ao);
        const Vec3f v2 = Cross(ao, co);

        const float v0d = Dot(v0, aRay.dir);
        const float v1d = Dot(v1, aRay.dir);
        const float v2d = Dot(v2, aRay.dir);

        if(((v0d < 0.f) & (v1d < 0.f) & (v2d < 0.f)) | ((v0d >= 0.f) & (v1d >= 0.f) & (v2d >= 0.f)))
        {
            const Vec3f normal   = Normalize(Cross(p[1] - p[0], p[2] - p[0]));
            const float distance = Dot(normal, ao) / Dot(normal, aRay.dir);

            if((distance > aRay.tmin) & (distance < oResult.dist))
            {
                oResult.normal = normal;
                oResult.matID  = matID;
                oResult.dist   = distance;
                return true;
            }
        }

        return false;
    }

    virtual void GrowBBox(Vec3f &aoBBoxMin, Vec3f &aoBBoxMax)
    {
        for(int i=0; i<3; i++)
            for(int j=0; j<3; j++)
            {
                aoBBoxMin.Get(j) = std::min(aoBBoxMin.Get(j), p[i].Get(j));
                aoBBoxMax.Get(j) = std::max(aoBBoxMax.Get(j), p[i].Get(j));
            }
    }
public:
    Vec3f p[3];
    int   matID;
};

class Sphere : public AbstractGeometry
{
public:
    Sphere(){}
    Sphere(const Vec3f& aCenter, float aRadius, int aMatID)
    {
        center = aCenter;
        radius = aRadius;
        matID  = aMatID;
    }

    // Taken from:
    // http://wiki.cgsociety.org/index.php/Ray_Sphere_Intersection
    virtual bool Intersect(const Ray& aRay, Isect& oResult) const
    {
        // we transform ray origin into object space (center == origin)
        const Vec3f transformedOrigin = aRay.org - center;

        const float A = Dot(aRay.dir, aRay.dir);
        const float B = 2 * Dot(aRay.dir, transformedOrigin);
        const float C = Dot(transformedOrigin, transformedOrigin) - (radius * radius);

        // Must use doubles, because when B ~ sqrt(B*B - 4*A*C)
        // the resulting t is imprecise enough to get around ray epsilons
        const double disc = B*B - 4*A*C;

        if(disc < 0)
            return false;

        const double discSqrt = std::sqrt(disc);
        const double q = (B < 0) ? ((-B - discSqrt) / 2.f) : ((-B + discSqrt) / 2.f);

        double t0 = q / A;
        double t1 = C / q;

        if(t0 > t1) std::swap(t0, t1);

        float resT;
        if(t0 > aRay.tmin && t0 < oResult.dist)
            resT = float(t0);
        else if(t1 > aRay.tmin && t1 < oResult.dist)
            resT = float(t1);
        else return false;

        oResult.dist   = resT;
        oResult.matID  = matID;
        oResult.normal = Normalize(transformedOrigin + Vec3f(resT) * aRay.dir);
        return true;
    }

    virtual void GrowBBox(Vec3f &aoBBoxMin, Vec3f &aoBBoxMax)
    {
        for(int i=0; i<8; i++)
        {
            Vec3f p = center;
            Vec3f half(radius);
            for(int j=0; j<3; j++)
                if(i & (1 << j)) half.Get(j) = -half.Get(j);
            p += half;

            for(int j=0; j<3; j++)
            {
                aoBBoxMin.Get(j) = std::min(aoBBoxMin.Get(j), p.Get(j));
                aoBBoxMax.Get(j) = std::max(aoBBoxMax.Get(j), p.Get(j));
            }
        }
    }
public:
    Vec3f center;
    float radius;
    int   matID;
};

#endif //__GEOMETRY_HXX__
