/*
 * This is published under Apache 2.0
 */

#ifndef __FRAME_HXX__
#define __FRAME_HXX__

#include <vector>
#include <cmath>
#include "math.hxx"

class Frame
{
public:
    Frame()
    {
        mX = Vec3f(1,0,0);
        mY = Vec3f(0,1,0);
        mZ = Vec3f(0,0,1);
    };
    Frame(
        const Vec3f& x,
        const Vec3f& y,
        const Vec3f& z)
        : mX(x),
        mY(y),
        mZ(z)
    {}

    void SetFromZ(const Vec3f& z)
    {
        Vec3f tmpZ = mZ = Normalize(z);
        Vec3f tmpX = (std::abs(tmpZ.x) > 0.99f) ? Vec3f(0,1,0) : Vec3f(1,0,0);
        mY = Normalize( Cross(tmpZ, tmpX) );
        mX = Cross(mY, tmpZ);
    }

    Vec3f ToWorld(const Vec3f& a) const
    {
        return mX * a.x + mY * a.y + mZ * a.z;
    }

    Vec3f ToLocal(const Vec3f& a) const
    {
        return Vec3f(Dot(a, mX), Dot(a, mY), Dot(a, mZ));
    }

    const Vec3f& Binormal() const { return mX; }
    const Vec3f& Tangent () const { return mY; }
    const Vec3f& Normal  () const { return mZ; }
public:
    Vec3f mX, mY, mZ;
};


#endif //__FRAME_HXX__
