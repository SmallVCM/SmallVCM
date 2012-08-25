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
    Scene() : mGeometry(NULL), mBackground(NULL) {};
    ~Scene()
    {
        delete mGeometry;
        for(size_t i=0; i<mLights.size(); i++)
            delete mLights[i];
    }

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
        isect.dist = aTMax - 2*EPS_RAY;

        return mGeometry->IntersectP(ray, isect);
    }

    const Material& GetMaterial(const int aMaterialIdx) const
    {
        return mMaterials[aMaterialIdx];
    }

    int GetMaterialCount() const
    {
        return (int)mMaterials.size();
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

    const BackgroundLight* GetBackground() const
    {
        return mBackground;
    }

    //////////////////////////////////////////////////////////////////////////
    // Loads a Cornell Box scene
    enum BoxMask
    {
        kLightCeiling    = 1,
        kLightSun        = 2,
        kLightPoint      = 4,
        kLightBackground = 8,
        kBallLargeMirror = 16,
        kBallLargeGlass  = 32,
        kBallMirror      = 64,
        kBallGlass       = 128,
        kGlossyFloor     = 256,
        kDefault         = (kLightCeiling | kBallMirror | kBallGlass),
        kBothSmallBalls  = (kBallMirror | kBallGlass),
        kBothLargeBalls  = (kBallLargeMirror | kBallLargeGlass)
    };
    void LoadCornellBox(const Vec2i &aResolution, uint aBoxMask = kDefault)
    {
        if((aBoxMask & kBothLargeBalls) == kBothLargeBalls)
        {
            printf("Cannot have both large balls, using mirror\n");
            aBoxMask &= ~kBallLargeGlass;
        }
        bool light_ceiling    = (aBoxMask & kLightCeiling)    != 0;
        bool light_sun        = (aBoxMask & kLightSun)        != 0;
        bool light_point      = (aBoxMask & kLightPoint)      != 0;
        bool light_background = (aBoxMask & kLightBackground) != 0;

        bool light_box = true;
        // because it looks really weird with it
        if(light_point)
            light_box = false;

        // Camera
        mCamera.Setup(
            Vec3f(-0.0439815f, -4.12529f, 0.222539f),
            Vec3f(0.00688625f, 0.998505f, -0.0542161f),
            Vec3f(3.73896e-4f, 0.0542148f, 0.998529f),
            Vec2f(float(aResolution.x), float(aResolution.y)), 45);

        // Materials
        Material mat;
        // 0) light1, will only emit
        mMaterials.push_back(mat);
        // 1) light2, will only emit
        mMaterials.push_back(mat);

        // 2) glossy white floor
        mat.Reset();
        mat.mDiffuseReflectance = Vec3f(0.1f);
        mat.mPhongReflectance   = Vec3f(0.7f);
        mat.mGlossiness         = 90.f;
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

        // 8) diffuse blue wall (back wall for glossy floor)
        mat.Reset();
        mat.mDiffuseReflectance = Vec3f(0.156863f, 0.172549f, 0.803922f);
        mMaterials.push_back(mat);

        delete mGeometry;

        //////////////////////////////////////////////////////////////////////////
        // Cornell box
        Vec3f cb[8] = {
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

        if((aBoxMask & kGlossyFloor) != 0)
        {
            // Floor
            geometryList->mGeometry.push_back(new Triangle(cb[0], cb[4], cb[5], 2));
            geometryList->mGeometry.push_back(new Triangle(cb[5], cb[1], cb[0], 2));
            // Back wall
            geometryList->mGeometry.push_back(new Triangle(cb[0], cb[1], cb[2], 8));
            geometryList->mGeometry.push_back(new Triangle(cb[2], cb[3], cb[0], 8));
        }
        else
        {
            // Floor
            geometryList->mGeometry.push_back(new Triangle(cb[0], cb[4], cb[5], 5));
            geometryList->mGeometry.push_back(new Triangle(cb[5], cb[1], cb[0], 5));
            // Back wall
            geometryList->mGeometry.push_back(new Triangle(cb[0], cb[1], cb[2], 5));
            geometryList->mGeometry.push_back(new Triangle(cb[2], cb[3], cb[0], 5));
        }


        // Ceiling
        if(light_ceiling && !light_box)
        {
            geometryList->mGeometry.push_back(new Triangle(cb[2], cb[6], cb[7], 0));
            geometryList->mGeometry.push_back(new Triangle(cb[7], cb[3], cb[2], 1));
        }
        else
        {
            geometryList->mGeometry.push_back(new Triangle(cb[2], cb[6], cb[7], 5));
            geometryList->mGeometry.push_back(new Triangle(cb[7], cb[3], cb[2], 5));
        }

        // Left wall
        geometryList->mGeometry.push_back(new Triangle(cb[3], cb[7], cb[4], 3));
        geometryList->mGeometry.push_back(new Triangle(cb[4], cb[0], cb[3], 3));

        // Right wall
        geometryList->mGeometry.push_back(new Triangle(cb[1], cb[5], cb[6], 4));
        geometryList->mGeometry.push_back(new Triangle(cb[6], cb[2], cb[1], 4));

        // Ball - central
        float largeRadius = 0.8f;
        Vec3f center = (cb[0] + cb[1] + cb[4] + cb[5]) * (1.f / 4.f) + Vec3f(0, 0, largeRadius);
        if((aBoxMask & kBallLargeMirror) != 0)
            geometryList->mGeometry.push_back(new Sphere(center, largeRadius, 6));
        if((aBoxMask & kBallLargeGlass) != 0)
            geometryList->mGeometry.push_back(new Sphere(center, largeRadius, 7));

        // Balls - left and right
        float smallRadius = 0.5f;
        Vec3f leftWallCenter  = (cb[0] + cb[4]) * (1.f / 2.f) + Vec3f(0, 0, smallRadius);
        Vec3f rightWallCenter = (cb[1] + cb[5]) * (1.f / 2.f) + Vec3f(0, 0, smallRadius);
        float xlen = rightWallCenter.x - leftWallCenter.x;
        Vec3f leftBallCenter  = leftWallCenter  + Vec3f(2.f * xlen / 7.f, 0, 0);
        Vec3f rightBallCenter = rightWallCenter - Vec3f(2.f * xlen / 7.f, 0, 0);
        if((aBoxMask & kBallMirror) != 0)
            geometryList->mGeometry.push_back(new Sphere(leftBallCenter,  smallRadius, 6));
        if((aBoxMask & kBallGlass) != 0)
            geometryList->mGeometry.push_back(new Sphere(rightBallCenter, smallRadius, 7));

        //////////////////////////////////////////////////////////////////////////
        // Light box at the ceiling
        Vec3f lb[8] = {
            Vec3f(-0.25f,  0.25f, 1.26002f),
            Vec3f( 0.25f,  0.25f, 1.26002f),
            Vec3f( 0.25f,  0.25f, 1.28002f),
            Vec3f(-0.25f,  0.25f, 1.28002f),
            Vec3f(-0.25f, -0.25f, 1.26002f),
            Vec3f( 0.25f, -0.25f, 1.26002f),
            Vec3f( 0.25f, -0.25f, 1.28002f),
            Vec3f(-0.25f, -0.25f, 1.28002f)
        };

        if(light_box)
        {
            // Back wall
            geometryList->mGeometry.push_back(new Triangle(lb[0], lb[2], lb[1], 5));
            geometryList->mGeometry.push_back(new Triangle(lb[2], lb[0], lb[3], 5));
            // Left wall
            geometryList->mGeometry.push_back(new Triangle(lb[3], lb[4], lb[7], 5));
            geometryList->mGeometry.push_back(new Triangle(lb[4], lb[3], lb[0], 5));
            // Right wall
            geometryList->mGeometry.push_back(new Triangle(lb[1], lb[6], lb[5], 5));
            geometryList->mGeometry.push_back(new Triangle(lb[6], lb[1], lb[2], 5));
            // Front wall
            geometryList->mGeometry.push_back(new Triangle(lb[4], lb[5], lb[6], 5));
            geometryList->mGeometry.push_back(new Triangle(lb[6], lb[7], lb[4], 5));

            if(light_ceiling)
            {
                // Floor
                geometryList->mGeometry.push_back(new Triangle(lb[0], lb[5], lb[4], 0));
                geometryList->mGeometry.push_back(new Triangle(lb[5], lb[0], lb[1], 1));
            }
            else
            {
                // Floor
                geometryList->mGeometry.push_back(new Triangle(lb[0], lb[5], lb[4], 5));
                geometryList->mGeometry.push_back(new Triangle(lb[5], lb[0], lb[1], 5));
            }
        }

        //////////////////////////////////////////////////////////////////////////
        // Lights
        if(light_ceiling && !light_box)
        {
            // Without lightbox, whole ceiling is light
            mLights.resize(2);
            AreaLight *l = new AreaLight(cb[2], cb[6], cb[7]);
            l->mIntensity = Vec3f(0.95492965f);
            mLights[0] = l;
            mMaterial2Light.insert(std::make_pair(0, 0));

            l = new AreaLight(cb[7], cb[3], cb[2]);
            l->mIntensity = Vec3f(0.95492965f);
            mLights[1] = l;
            mMaterial2Light.insert(std::make_pair(1, 1));
        }
        else if(light_ceiling && light_box)
        {
            // With lightbox
            mLights.resize(2);
            AreaLight *l = new AreaLight(lb[0], lb[5], lb[4]);
            //l->mIntensity = Vec3f(0.95492965f);
            l->mIntensity = Vec3f(25.03329895614464f);
            mLights[0] = l;
            mMaterial2Light.insert(std::make_pair(0, 0));

            l = new AreaLight(lb[5], lb[0], lb[1]);
            //l->mIntensity = Vec3f(0.95492965f);
            l->mIntensity = Vec3f(25.03329895614464f);
            mLights[1] = l;
            mMaterial2Light.insert(std::make_pair(1, 1));
        }

        if(light_sun)
        {
            DirectionalLight *l = new DirectionalLight(Vec3f(-1.f, 1.5f, -1.f));
            l->mIntensity = Vec3f(0.5f, 0.2f, 0.f) * 1.5f;
            mLights.push_back(l);
        }

        if(light_point)
        {
            PointLight *l = new PointLight(Vec3f(0.0, -0.5, 1.0));
            l->mIntensity = Vec3f(70.f * (INV_PI_F * 0.25f));
            mLights.push_back(l);
        }

        if(light_background)
        {
            BackgroundLight *l = new BackgroundLight;
            l->mScale = 1.f;
            mLights.push_back(l);
            mBackground = l;
        }
    }

    void BuildSceneSphere()
    {
        Vec3f bboxMin( 1e36f);
        Vec3f bboxMax(-1e36f);
        mGeometry->GrowBBox(bboxMin, bboxMax);

        const float radius2 = (bboxMax - bboxMin).LenSqr();

        mSceneSphere.mSceneCenter = (bboxMax + bboxMin) * 0.5f;
        mSceneSphere.mSceneRadius = std::sqrt(radius2) * 0.5f;
        mSceneSphere.mInvSceneRadiusSqr = 1.f / Sqr(mSceneSphere.mSceneRadius);
    }
private:

public:
    AbstractGeometry      *mGeometry;
    Camera                mCamera;
    std::vector<Material> mMaterials;
    std::vector<AbstractLight*>   mLights;
    std::map<int, int> mMaterial2Light;
    SceneSphere        mSceneSphere;
    BackgroundLight*   mBackground;
};

#endif //__SCENE_HXX__
