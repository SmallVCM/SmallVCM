/*
 * This is published under Apache 2.0
 */

#ifndef __SCENE_HXX__
#define __SCENE_HXX__

#include <vector>
#include <cmath>
#include "math.hxx"
#include "geometry.hxx"
#include "camera.hxx"
#include "materials.hxx"
#include "lights.hxx"

class Scene
{
public:
    Scene() : mGeometry(NULL){};
    ~Scene() { delete mGeometry; }

    bool Intersect(const Ray& aRay, Isect& oResult) const
    {
        return mGeometry->Intersect(aRay, oResult);
    }

    void LoadCornellBox()
    {
        // Camera
        mCamera.Setup(
            Vec3f(-0.0439815f, -4.12529f, 0.222539f),
            Vec3f(0.00688625f, 0.998505f, -0.0542161f),
            Vec3f(3.73896e-4f, 0.0542148f, 0.998529f),
            Vec2f(32, 32), 45);

        delete mGeometry;
        Vec3f p[8] = {
            Vec3f(-1.27029f,  1.30455f, -1.28002f),
            Vec3f( 1.28975f,  1.30455f, -1.28002f),
            Vec3f( 1.28975f,  1.30455f,  1.28002f),
            Vec3f(-1.27029f,  1.30455f,  1.28002f),
            Vec3f(-1.27029f, -1.25549f, -1.28002f),
            Vec3f( 1.28975f, -1.25549f, -1.28002f),
            Vec3f( 1.28975f, -1.25549f,  1.28002f),
            Vec3f(-1.27029f, -1.25549f,  1.28002f)
        };

        GeometryList *geometryList = new GeometryList;
        mGeometry = geometryList;

        // Floor
        geometryList->mGeometry.push_back(new Triangle(p[0], p[4], p[5], 0));
        geometryList->mGeometry.push_back(new Triangle(p[5], p[1], p[0], 1));

        // Back wall
        geometryList->mGeometry.push_back(new Triangle(p[0], p[1], p[2], 2));
        geometryList->mGeometry.push_back(new Triangle(p[2], p[3], p[0], 3));

        // Ceiling
        geometryList->mGeometry.push_back(new Triangle(p[2], p[6], p[7], 4));
        geometryList->mGeometry.push_back(new Triangle(p[7], p[3], p[2], 5));

        // Left wall
        geometryList->mGeometry.push_back(new Triangle(p[3], p[7], p[4], 6));
        geometryList->mGeometry.push_back(new Triangle(p[4], p[0], p[3], 7));

        // Right wall
        geometryList->mGeometry.push_back(new Triangle(p[1], p[5], p[6], 8));
        geometryList->mGeometry.push_back(new Triangle(p[6], p[2], p[1], 9));

        // Ball - central
        float radius = 0.4f;
        Vec3f center = (p[0] + p[1] + p[4] + p[5]) * Vec3f(1.f / 4) + Vec3f(0, 0, radius);
        geometryList->mGeometry.push_back(new Sphere(center, radius, 10));
    }
public:
    AbstractGeometry      *mGeometry;
    Camera                mCamera;
    std::vector<Material> mMaterials;
    std::vector<Light>    mLights;
};

#endif //__SCENE_HXX__
