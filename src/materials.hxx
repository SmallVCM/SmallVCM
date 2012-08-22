/*
 * This is published under Apache 2.0
 */

#ifndef __MATERIALS_HXX__
#define __MATERIALS_HXX__

#include <vector>
#include <cmath>
#include "math.hxx"
#include "frame.hxx"
#include "ray.hxx"
#include "scene.hxx"
#include "utils.hxx"

class Material
{
public:
    Material()
    {
        Reset();
    }

    void Reset()
    {
        mDiffuseReflectance = Vec3f(0);
        mPhongReflectance   = Vec3f(0);
        mGlossiness         = 1.f;
        mMirrorReflectance  = Vec3f(0);
        mIOR = -1.f;
    }

    // diffuse is simply added to the others
    Vec3f mDiffuseReflectance;
    // phong is simply added to the others
    Vec3f mPhongReflectance;
    float mGlossiness;

    // mirror can be either simply added, or mixed using fresnel term
    // this is governed by mIOR, if it is >= 0, fresnel is used, otherwise
    // it is not
    Vec3f mMirrorReflectance;

    // When mIOR >= 0, we also transmit (just clear glass)
    float mIOR;
};

#endif //__MATERIALS_HXX__
