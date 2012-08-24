/*
 * This is published under Apache 2.0
 */

#ifndef __FRAMEBUFFER_HXX__
#define __FRAMEBUFFER_HXX__

#include <vector>
#include <cmath>
#include <fstream>

class Framebuffer
{
public:
    Framebuffer(){};

    //////////////////////////////////////////////////////////////////////////
    // Accumulation
    void AddColor(const Vec2f& aSample, const Vec3f& aColor)
    {
        if(aSample.x < 0 || aSample.x >= mResolution.x) return;
        if(aSample.y < 0 || aSample.y >= mResolution.y) return;

        int x = int(aSample.x);
        int y = int(aSample.y);

        mColor[x + y * mResX] = mColor[x + y * mResX] + aColor;
    }

    //////////////////////////////////////////////////////////////////////////
    // Methods for framebuffer operations
    void Setup(const Vec2f& aResolution)
    {
        mResolution = aResolution;
        mResX = int(aResolution.x);
        mResY = int(aResolution.y);
        mColor.resize(mResX * mResY);
        Clear();
    }

    void Clear()
    {
        memset(&mColor[0], 0, sizeof(Vec3f) * mColor.size());
    }

    void Add(const Framebuffer& aOther)
    {
        for(size_t i=0; i<mColor.size(); i++)
            mColor[i] = mColor[i] + aOther.mColor[i];
    }

    void Scale(float aScale)
    {
        for(size_t i=0; i<mColor.size(); i++)
            mColor[i] = mColor[i] * Vec3f(aScale);
    }

    void SavePPM(const char* aFilename, float aGamma = 1.f)
    {
        const float invGamma = 1.f / aGamma;

        std::ofstream ppm(aFilename);
        ppm << "P3" << std::endl;
        ppm << mResX << " " << mResY << std::endl;
        ppm << "255" << std::endl;

        Vec3f *ptr = &mColor[0];
        for(int y=0; y<mResY; y++)
        {
            for(int x=0; x<mResX; x++)
            {
                ptr = &mColor[x + (mResY-y-1)*mResX];
                int r = int(std::pow(ptr->x, invGamma) * 255.f);
                int g = int(std::pow(ptr->y, invGamma) * 255.f);
                int b = int(std::pow(ptr->z, invGamma) * 255.f);

                ppm << std::min(255, std::max(0, r)) << " "
                    << std::min(255, std::max(0, g)) << " "
                    << std::min(255, std::max(0, b)) << " ";
            }
            ppm << std::endl;
        }
    }

    void SavePFM(const char* aFilename)
    {
        std::ofstream ppm(aFilename, std::ios::binary);
        ppm << "PF" << std::endl;
        ppm << mResX << " " << mResY << std::endl;
        ppm << "-1" << std::endl;

        ppm.write(reinterpret_cast<const char*>(&mColor[0]),
            mColor.size() * sizeof(Vec3f));
    }
private:
    std::vector<Vec3f> mColor;
    Vec2f              mResolution;
    int                mResX;
    int                mResY;
};

#endif //__FRAMEBUFFER_HXX__
