/*
 * This is published under Apache 2.0
 */

#ifndef __SCENE_HXX__
#define __SCENE_HXX__

#include <vector>
#include <map>
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
        bool hit = mGeometry->Intersect(aRay, oResult);
        if(hit)
        {
            oResult.lightID = -1;
            std::map<int, int>::const_iterator it =
                mMaterial2Light.find(oResult.matID);

            if(it != mMaterial2Light.end())
                oResult.lightID = it->second;
        }
        return hit;
    }

    bool Occluded(const Vec3f& aPoint, const Vec3f& aDir, float aTMax) const
    {
        Ray ray;
        ray.org  = aPoint + aDir * EPS_RAY;
        ray.dir  = aDir;
        ray.tmin = 0;
        Isect isect;
        isect.dist = aTMax - EPS_RAY;

        return mGeometry->IntersectP(ray, isect);
    }

    const Material& GetMaterial(const int aMaterialIdx) const
    {
        return mMaterials[aMaterialIdx];
    }

    const AbstractLight* GetLightPtr(int aLightIdx) const
    {
        aLightIdx = std::min<int>(aLightIdx, mLights.size()-1);
        return mLights[aLightIdx];
    }

    int GetLightCount() const
    {
        return (int)mLights.size();
    }

    //////////////////////////////////////////////////////////////////////////
    // Loads a Cornell Box scene
    void LoadCornellBox()
    {
        // Camera
        mCamera.Setup(
            Vec3f(-0.0439815f, -4.12529f, 0.222539f),
            Vec3f(0.00688625f, 0.998505f, -0.0542161f),
            Vec3f(3.73896e-4f, 0.0542148f, 0.998529f),
            Vec2f(256, 256), 45);

        // Materials
        Material mat;
        // 0) light1, will only emit
        mMaterials.push_back(mat);
        // 1) light2, will only emit
        mMaterials.push_back(mat);

        // 2) glossy white floor
        mat.Reset();
        mat.mDiffuseReflectance = Vec3f(0.3f);
        mat.mPhongReflectance   = Vec3f(0.4f);
        mat.mGlossiness         = 10.f;
        mMaterials.push_back(mat);

        // 3) diffuse green left wall
        mat.Reset();
        mat.mDiffuseReflectance = Vec3f(0.156863f, 0.803922f, 0.172549f);
        mMaterials.push_back(mat);

        // 4) diffuse red right wall
        mat.Reset();
        mat.mDiffuseReflectance = Vec3f(0.803922f, 0.152941f, 0.152941f);
        mMaterials.push_back(mat);

        // 5) diffuse white back wall
        mat.Reset();
        mat.mDiffuseReflectance = Vec3f(0.803922f, 0.803922f, 0.803922f);
        mMaterials.push_back(mat);

        // 6) mirror ball
        mat.Reset();
        mat.mMirrorReflectance = Vec3f(1.f);
        mMaterials.push_back(mat);

        // 7) glass ball
        mat.Reset();
        mat.mMirrorReflectance  = Vec3f(1.f);
        mat.mIOR                = 1.6f;
        mMaterials.push_back(mat);

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
        geometryList->mGeometry.push_back(new Triangle(p[0], p[4], p[5], 5));
        geometryList->mGeometry.push_back(new Triangle(p[5], p[1], p[0], 5));

        // Back wall
        geometryList->mGeometry.push_back(new Triangle(p[0], p[1], p[2], 5));
        geometryList->mGeometry.push_back(new Triangle(p[2], p[3], p[0], 5));

        // Ceiling
        geometryList->mGeometry.push_back(new Triangle(p[2], p[6], p[7], 0));
        geometryList->mGeometry.push_back(new Triangle(p[7], p[3], p[2], 1));

        // Left wall
        geometryList->mGeometry.push_back(new Triangle(p[3], p[7], p[4], 3));
        geometryList->mGeometry.push_back(new Triangle(p[4], p[0], p[3], 3));

        // Right wall
        geometryList->mGeometry.push_back(new Triangle(p[1], p[5], p[6], 4));
        geometryList->mGeometry.push_back(new Triangle(p[6], p[2], p[1], 4));

        // Ball - central
        float radius = 0.5f;
        Vec3f center = (p[0] + p[1] + p[4] + p[5]) * (1.f / 4.f) + Vec3f(0, 0, radius);
        //geometryList->mGeometry.push_back(new Sphere(center, radius, 6));

        // Balls - left and right
        Vec3f leftWallCenter  = (p[0] + p[4]) * (1.f / 2.f) + Vec3f(0, 0, radius);
        Vec3f rightWallCenter = (p[1] + p[5]) * (1.f / 2.f) + Vec3f(0, 0, radius);
        float xlen = rightWallCenter.x - leftWallCenter.x;
        Vec3f leftBallCenter  = leftWallCenter  + Vec3f(2.f * xlen / 7.f, 0, 0);
        Vec3f rightBallCenter = rightWallCenter - Vec3f(2.f * xlen / 7.f, 0, 0);
        //geometryList->mGeometry.push_back(new Sphere(leftBallCenter,  radius, 6));
        //geometryList->mGeometry.push_back(new Sphere(rightBallCenter, radius, 7));

        // Lights
        mLights.resize(2);
        AreaLight *l = new AreaLight(p[2], p[6], p[7]);
        l->mIntensity = Vec3f(0.95492965f);
        mLights[0] = l;

        l = new AreaLight(p[7], p[3], p[2]);
        l->mIntensity = Vec3f(0.95492965f);
        mLights[1] = l;

        mMaterial2Light.insert(std::make_pair(0, 0));
        mMaterial2Light.insert(std::make_pair(1, 1));
    }
public:
    AbstractGeometry      *mGeometry;
    Camera                mCamera;
    std::vector<Material> mMaterials;
    std::vector<AbstractLight*>   mLights;
    std::map<int, int> mMaterial2Light;
};

#endif //__SCENE_HXX__
