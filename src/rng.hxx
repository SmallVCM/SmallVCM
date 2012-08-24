/*
 * This is published under Apache 2.0
 */

#ifndef __RNG_HXX__
#define __RNG_HXX__

#include <vector>
#include <cmath>
#include <random>
#include "renderer.hxx"
#include "bxdf.hxx"

class Rng
{
public:
    Rng(int aSeed = 1234):
        mRng(aSeed)
    {}

    int GetInt()
    {
        return mDistInt(mRng);
    }

    uint GetUint()
    {
        return mDistUint(mRng);
    }

    float GetFloat()
    {
        return mDistFloat(mRng);
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
    std::mt19937_64 mRng;
    std::uniform_int_distribution<int>    mDistInt;
    std::uniform_int_distribution<uint>   mDistUint;
    std::uniform_real_distribution<float> mDistFloat;
};

namespace TeaDetail_v2
{
    int JenkinsMix(uint a, uint b, uint c)
    {
        a=a-b;  a=a-c;  a=a^(c >> 13);
        b=b-c;  b=b-a;  b=b^(a << 8);
        c=c-a;  c=c-b;  c=c^(b >> 13);
        a=a-b;  a=a-c;  a=a^(c >> 12);
        b=b-c;  b=b-a;  b=b^(a << 16);
        c=c-a;  c=c-b;  c=c^(b >> 5);
        a=a-b;  a=a-c;  a=a^(c >> 3);
        b=b-c;  b=b-a;  b=b^(a << 10);
        c=c-a;  c=c-b;  c=c^(b >> 15);
        return c;
    }

    template<unsigned int rounds>
    class TeaQImplTemplate_v2
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // aCpuRndSeed  - random seed generated on CPU, not used for Sobol
        // aIndexOffset - offset to be added to g_IndexBase to get real index
        // aPixelIndex   - a number based on pixel/slot, used to scramble
        void Reset(uint aCpuRndSeed, uint aIndexOffset, uint aPixelIndex)
        {
            mState0 = aCpuRndSeed;
            mState1 = JenkinsMix(aIndexOffset, aPixelIndex, 0x9e3779b9u);
        }

        void ForceNextDimension()                   {}
        void ForceDimensionBy(int /*aDimensionOffset*/) {}

        uint GetImpl(void)
        {
            unsigned int sum=0;
            const unsigned int delta=0x9e3779b9U;

            for (unsigned int i=0; i<rounds; i++) {
                sum+=delta;
                mState0+=((mState1<<4)+0xa341316cU) ^ (mState1+sum) ^ ((mState1>>5)+0xc8013ea4U);
                mState1+=((mState0<<4)+0xad90777dU) ^ (mState0+sum) ^ ((mState0>>5)+0x7e95761eU);
            }
            return mState0;
        }
        //////////////////////////////////////////////////////////////////////////
        void StoreState(uint *oState1, uint *oState2)
        {
            *oState1 = mState0;
            *oState2 = mState1;
        }

        void LoadState(uint aState1, uint aState2, uint /*aDimension*/)
        {
            mState0 = aState1;
            mState1 = aState2;
        }

    private:
        uint mState0, mState1;
    };
}

typedef TeaDetail_v2::TeaQImplTemplate_v2<6>  TeaQImpl_v2;

template<typename GpuQRandomImpl>
class GpuQRandomBase_v2
{
public:
    //////////////////////////////////////////////////////////////////////////
    // aCpuRndSeed  - random seed generated on CPU, not used for Sobol
    // aIndexOffset - offset to be added to g_IndexBase to get real index
    // aPixelIndex   - a number based on pixel/slot, used to scramble
    void Reset(uint aCpuRndSeed, uint aIndexOffset, uint aPixelIndex)
    {
        mImpl.Reset(aCpuRndSeed, aIndexOffset, aPixelIndex);
    }

    void ForceNextDimension()                   { mImpl.ForceNextDimension(); }
    void ForceDimensionBy(int aDimensionOffset) { mImpl.ForceDimensionBy(aDimensionOffset); }

    uint  GetUint   (void)            { return getImpl(); }
    float GetFloat  (void)            { return (float(GetUint()) + 1.f) * (1.0f / 4294967297.0f); }

    Vec2f GetVec2f   (void)
    {
        // cannot do return Vec2f(getF32(), getF32()) because the order is not ensured
        float a = GetFloat();
        float b = GetFloat();
        return Vec2f(a, b);
    }
    Vec3f GetVec3f   (void)
    {
        float a = GetFloat();
        float b = GetFloat();
        float c = GetFloat();
        return Vec3f(a, b, c);
    }

    //////////////////////////////////////////////////////////////////////////
    void StoreState(uint *oState1, uint *oState2)
    {
        mImpl.StoreState(oState1, oState2);
    }

    void LoadState(uint aState1, uint aState2, uint aDimension)
    {
        mImpl.LoadState(aState1, aState2, aDimension);
    }

protected:
    uint getImpl(void) { return mImpl.GetImpl(); }

private:
    GpuQRandomImpl mImpl;
};

typedef GpuQRandomBase_v2<TeaQImpl_v2> GpuQ2RandomTea;

#endif //__RNG_HXX__
