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

#ifndef __SOBOL_HXX__
#define __SOBOL_HXX__

#include <vector>
#include <cmath>
#include <random>

class BaseSobol
{
public:
    enum SobolMode
    {
        // when there is only single traced path, like PT or LT
        kIsSingle,
        // used for any kind of bidirectional method, BPT, BPM, PM, VCM
        kIsCamera,
        kIsLight,
    };
public:
    BaseSobol(int aSeed = 1234)
        : mMatrices(52, 1024),
        mRng(aSeed)
    {
        mScramble = 0;
        mMatrices.GenerateMatrices();
    }

    //////////////////////////////////////////////////////////////////////////
    // Specific Quasi-random methods

    // Resets
    void ResetAnyPixel(uint64 aIndex, SobolMode aMode)
    {
        mIndex = aIndex;
        // Dimensions 0 and 1 are always reserved for image plane sampling,
        // and are sampled explicitly.
        // For single sampling, we simply continue on.
        // When we have both camera and light paths, then next camera sample
        // is 3, 5, 7, 9... light samples are 2, 4, 6, 8..
        switch(aMode)
        {
        case kIsSingle:
            mDimension = 2;
            mDimensionIncrement = 1;
            break;
        case kIsCamera:
            mDimension = 3;
            mDimensionIncrement = 2;
            break;
        case kIsLight:
            mDimension = 2;
            mDimensionIncrement = 2;
            break;
        }
    }


    void ResetGivenPixel(SobolMode /*aMode*/,
        const Vec2i& /*aPixelCoords*/,
        const Vec2i& /*aResolution*/)
    {
    }


    Vec2f GetCameraSample(const Vec2i& aRes)
    {
        float x01 = BaseUint(0) * (1.f / (uint(1) << 32));
        float y01 = BaseUint(1) * (1.f / (uint(1) << 32));
        Vec2f sample = Vec2f(x01 * aRes.x, y01 * aRes.y);

    }

    void AdvanceDimensionBy(const uint aAdvanceBy)
    {
        mDimension += mDimensionIncrement * aAdvanceBy;
    }

    //////////////////////////////////////////////////////////////////////////
    // Default RNG methods
    int GetInt()
    {
        return int(GetUint());
    }

    uint GetUint()
    {
        uint result = BaseUint();
        mDimension += mDimensionIncrement;
        return result;
    }

    float GetFloat()
    {
        return GetUint() * (1.f / (uint(1) << 32));
    }

    Vec2f GetVec2f()
    {
        float a = GetFloat();
        float b = GetFloat();
        return Vec2f(a, b);
    }

    Vec3f GetVec3f()
    {
        float a = GetFloat();
        float b = GetFloat();
        float c = GetFloat();
        return Vec3f(a, b, c);
    }
private:
    uint BaseUint(uint aDimension)
    {
        if(aDimension < mMatrices.mNumberOfDimensions)
            return mMatrices.Sample32(mIndex, aDimension, mScramble);
        return mDistUint(mRng);
    }
private:
    uint64          mIndex;
    uint            mScramble;
    uint            mDimension;
    uint            mDimensionIncrement;

    sobol::Matrices mMatrices;
    // For backup, if we run out of dimensions
    std::mt19937_64 mRng;
    std::uniform_int_distribution<uint>   mDistUint;
};


#endif //__SOBOL_HXX__
