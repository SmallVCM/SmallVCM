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

    //////////////////////////////////////////////////////////////////////////
    // Saving
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
        //to avoid alignment issues, we leave this outside
        //char   mMagic[2];        //!< always: BM
        uint   mFileSize;        //!< size of file in bytes
        uint   mReserved01;      //!< 2x 2 reserved bytes
        uint   mDataOffset;      //!< Offset in bytes where data can be found (54)

        uint   mHeaderSize;      //!< 40B
        int    mWidth;           //!< width in pixels
        int    mHeight;          //!< height in pixels

        short  mColorPlates;     //!< must be 1
        short  mBitsPerPixel;    //!< we use 24bpp
        uint   mCompression;     //!< we use BI_RGB ~ 0, uncompressed
        uint   mImageSize;       //!< mWidth x mHeight x 3B
        uint   mHorizRes;        //!< pixels per meter (75dpi ~ 2953ppm)
        uint   mVertRes;         //!< pixels per meter (75dpi ~ 2953ppm)
        uint   mPaletteColors;   //!< not using palette - 0
        uint   mImportantColors; //!< 0 - all are important
    };

    void SaveBMP(const char* aFilename, float aGamma = 1.f)
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
                byte bgrB[3];
                bgrB[0] = byte(std::pow(rgbF.z, invGamma) * 255.f);
                bgrB[1] = byte(std::pow(rgbF.y, invGamma) * 255.f);
                bgrB[2] = byte(std::pow(rgbF.x, invGamma) * 255.f);

                bmp.write((char*)&bgrB, sizeof(bgrB));
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
