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
#include "sobol_core.hxx"

class BaseSobol
{
public:
    enum SobolMode
    {
        // When tracing only camera or only light paths
        kCameraOnly,
        kLightOnly,
        // When tracing camera and light paths (BPT, VCM, PM)
        kCameraInCombined,
        kLightInCombined
    };
public:
    BaseSobol(int aSeed = 1234, uint mMaxDimension = 1024)
        : mMatrices(52, mMaxDimension),
        mRng(aSeed)
    {
        mScramble = 0;
        mMatrices.GenerateMatrices();
    }

    //////////////////////////////////////////////////////////////////////////
    // Specific Quasi-random methods
    void SetupForResolution(const Vec2i& aResolution)
    {
        mResolution = aResolution;

        int maxRes = std::max(aResolution.x, aResolution.y);
        int maxIdx = maxRes - 1;
        for(mBitsForPixel = 0; maxIdx; maxIdx >>= 1, mBitsForPixel++);
        mIndicesPerIteration = uint64(1) << uint64(mBitsForPixel << 1);
    }

    // Determines mIndex by iteration and path index
    void ResetByIndex(uint aIteration, uint aIterIndex, SobolMode aMode)
    {
        mUseExplicitPixel = false;
        mIndex = uint64(aIteration) * mIndicesPerIteration + aIterIndex;

        // Dimensions 0 and 1 are always reserved for image plane sampling,
        // and are sampled explicitly.
        // For single sampling, we simply continue on.
        // When we have both camera and light paths, then next camera sample
        // is 3, 5, 7, 9... light samples are 2, 4, 6, 8..
        switch(aMode)
        {
        case kCameraOnly:
            mDimension = 2;
            mDimensionIncrement = 1;
            break;
        case kLightOnly: // without special camera handling
            mDimension = 0;
            mDimensionIncrement = 1;
            break;
        case kCameraInCombined:
            mDimension = 3;
            mDimensionIncrement = 2;
            break;
        case kLightInCombined:
            mDimension = 2;
            mDimensionIncrement = 2;
            break;
        }
    }

    // Determines mIndex by iteration and requested pixel coordinates
    void ResetByPixel(uint /*aIteration*/, Vec2i /*aPixelCoords*/, SobolMode /*aMode*/)
    {
        mUseExplicitPixel = true;
    }


    Vec2f GetCameraSample()
    {
        if(!mUseExplicitPixel)
        {
            float x01 = BaseUint(0) * (1.f / (1ull << 32));
            float y01 = BaseUint(1) * (1.f / (1ull << 32));
            return Vec2f(x01 * mResolution.x, y01 * mResolution.y);
        }
        else
        {
            float jitterX = BaseUint(0) * (1.f / (1ull << (32u - mBitsForPixel) ));
            float jitterY = BaseUint(1) * (1.f / (1ull << (32u - mBitsForPixel) ));
            return Vec2f(mExplicitPixelCoords.x + jitterX,
                mExplicitPixelCoords.y + jitterY);
        }
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
        uint result = BaseUint(mDimension);
        mDimension += mDimensionIncrement;
        return result;
    }

    float GetFloat()
    {
        return GetUint() * (1.f / (1ull << 32));
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
    // When we explicitly specify pixel to use
    bool            mUseExplicitPixel;
    Vec2i           mExplicitPixelCoords;

    uint64          mIndex;
    uint            mScramble;
    uint            mDimension;
    uint            mDimensionIncrement;

    Vec2i           mResolution;
    // How many bits are used to determine which pixel index belongs to.
    // This is m in http://gruenschloss.org/sample-enum/sample-enum-src.zip
    uint            mBitsForPixel;
    // 2^(mBitsForPixel*2)
    uint64          mIndicesPerIteration;

    sobol::Matrices mMatrices;
    // For backup, if we run out of dimensions
    std::mt19937_64 mRng;
    std::uniform_int_distribution<uint>   mDistUint;
};


#endif //__SOBOL_HXX__
