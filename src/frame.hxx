/*
 * Copyright (C) 2012, Tomas Davidovic (http://www.davidovic.cz)
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom
 * the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
 * OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * (The above is MIT License: http://en.wikipedia.org/wiki/MIT_License)
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
