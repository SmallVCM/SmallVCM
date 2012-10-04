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

#ifndef __FRAMEBUFFER_HXX__
#define __FRAMEBUFFER_HXX__

#include <vector>
#include <cmath>
#include <fstream>
#include <string.h>
#include "utils.hxx"

class Framebuffer
{
public:

    Framebuffer()
    {}

    //////////////////////////////////////////////////////////////////////////
    // Accumulation
    void AddColor(
        const Vec2f& aSample,
        const Vec3f& aColor)
    {
        if(aSample.x < 0 || aSample.x >= mResolution.x)
            return;

        if(aSample.y < 0 || aSample.y >= mResolution.y)
            return;

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

    //////////////////////////////////////////////////////////////////////////
    // Statistics
    float TotalLuminance()
    {
        float lum = 0;

        for(int y=0; y<mResY; y++)
        {
            for(int x=0; x<mResX; x++)
            {
                lum += Luminance(mColor[x + y*mResX]);
            }
        }

        return lum;
    }

    //////////////////////////////////////////////////////////////////////////
    // Saving
    void SavePPM(
        const char *aFilename,
        float       aGamma = 1.f)
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
                ptr = &mColor[x + y*mResX];
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

    //////////////////////////////////////////////////////////////////////////
    // Saving BMP
    struct BmpHeader
    {
        uint   mFileSize;        // Size of file in bytes
        uint   mReserved01;      // 2x 2 reserved bytes
        uint   mDataOffset;      // Offset in bytes where data can be found (54)

        uint   mHeaderSize;      // 40B
        int    mWidth;           // Width in pixels
        int    mHeight;          // Height in pixels

        short  mColorPlates;     // Must be 1
        short  mBitsPerPixel;    // We use 24bpp
        uint   mCompression;     // We use BI_RGB ~ 0, uncompressed
        uint   mImageSize;       // mWidth x mHeight x 3B
        uint   mHorizRes;        // Pixels per meter (75dpi ~ 2953ppm)
        uint   mVertRes;         // Pixels per meter (75dpi ~ 2953ppm)
        uint   mPaletteColors;   // Not using palette - 0
        uint   mImportantColors; // 0 - all are important
    };

    void SaveBMP(
        const char *aFilename,
        float       aGamma = 1.f)
    {
        std::ofstream bmp(aFilename, std::ios::binary);
        BmpHeader header;
        bmp.write("BM", 2);
        header.mFileSize   = uint(sizeof(BmpHeader) + 2) + mResX * mResX * 3;
        header.mReserved01 = 0;
        header.mDataOffset = uint(sizeof(BmpHeader) + 2);
        header.mHeaderSize = 40;
        header.mWidth      = mResX;
        header.mHeight     = mResY;
        header.mColorPlates     = 1;
        header.mBitsPerPixel    = 24;
        header.mCompression     = 0;
        header.mImageSize       = mResX * mResY * 3;
        header.mHorizRes        = 2953;
        header.mVertRes         = 2953;
        header.mPaletteColors   = 0;
        header.mImportantColors = 0;

        bmp.write((char*)&header, sizeof(header));

        const float invGamma = 1.f / aGamma;
        for(int y=0; y<mResY; y++)
        {
            for(int x=0; x<mResX; x++)
            {
                // bmp is stored from bottom up
                const Vec3f &rgbF = mColor[x + (mResY-y-1)*mResX];
                typedef unsigned char byte;
                float gammaBgr[3];
                gammaBgr[0] = std::pow(rgbF.z, invGamma) * 255.f;
                gammaBgr[1] = std::pow(rgbF.y, invGamma) * 255.f;
                gammaBgr[2] = std::pow(rgbF.x, invGamma) * 255.f;

                byte bgrB[3];
                bgrB[0] = byte(std::min(255.f, std::max(0.f, gammaBgr[0])));
                bgrB[1] = byte(std::min(255.f, std::max(0.f, gammaBgr[1])));
                bgrB[2] = byte(std::min(255.f, std::max(0.f, gammaBgr[2])));

                bmp.write((char*)&bgrB, sizeof(bgrB));
            }
        }
    }

    //////////////////////////////////////////////////////////////////////////
    // Saving HDR
    void SaveHDR(const char* aFilename)
    {
        std::ofstream hdr(aFilename, std::ios::binary);

        hdr << "#?RADIANCE" << '\n';
        hdr << "# SmallVCM" << '\n';
        hdr << "FORMAT=32-bit_rle_rgbe" << '\n' << '\n';
        hdr << "-Y " << mResY << " +X " << mResX << '\n';

        for(int y=0; y<mResY; y++)
        {
            for(int x=0; x<mResX; x++)
            {
                typedef unsigned char byte;
                byte rgbe[4] = {0,0,0,0};

                const Vec3f &rgbF = mColor[x + y*mResX];
                float v = std::max(rgbF.x, std::max(rgbF.y, rgbF.z));

                if(v >= 1e-32f)
                {
                    int e;
                    v = float(frexp(v, &e) * 256.f / v);
                    rgbe[0] = byte(rgbF.x * v);
                    rgbe[1] = byte(rgbF.y * v);
                    rgbe[2] = byte(rgbF.z * v);
                    rgbe[3] = byte(e + 128);
                }

                hdr.write((char*)&rgbe[0], 4);
            }
        }
    }

private:

    std::vector<Vec3f> mColor;
    Vec2f              mResolution;
    int                mResX;
    int                mResY;
};

#endif //__FRAMEBUFFER_HXX__
