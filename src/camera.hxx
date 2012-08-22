/*
 * This is published under Apache 2.0
 */

#ifndef __CAMERA_HXX__
#define __CAMERA_HXX__

#include <vector>
#include <cmath>
#include "math.hxx"
#include "ray.hxx"

//////////////////////////////////////////////////////////////////////////
// Camera

class Camera
{
public:
    void Setup(const Vec3f& aPosition, const Vec3f& aForward, const Vec3f& aUp,
        const Vec2f& aResolution, float aHorizontalFOV)
    {
        const Vec3f forward = Normalize(aForward);
        const Vec3f up      = -Normalize(Cross(aUp, -forward));
        const Vec3f left    = Cross(-forward, up);

        mPosition   = aPosition;
        mResolution = aResolution;

        const Vec3f pos(
            Dot(up, aPosition),
            Dot(left, aPosition),
            Dot(-forward, aPosition));

        Mat4f worldToCamera = Mat4f::Indetity();
        worldToCamera.SetRow(0, up,       -pos.x);
        worldToCamera.SetRow(1, left,     -pos.y);
        worldToCamera.SetRow(2, -forward, -pos.z);

        const Mat4f perspective = Mat4f::Perspective(aHorizontalFOV, 0.1f, 10000.f);

        const Mat4f worldToNScreen = perspective * worldToCamera;
        const Mat4f nscreenToWorld = Invert(worldToNScreen);

        mWorldToRaster  =
            Mat4f::Scale(Vec3f(aResolution.x * 0.5f, aResolution.y * 0.5f, 0)) *
            Mat4f::Translate(Vec3f(1.f, 1.f, 0)) * worldToNScreen;

        mRasterToWorld  = nscreenToWorld *
            Mat4f::Translate(Vec3f(-1.f, -1.f, 0)) *
            Mat4f::Scale(Vec3f(2.f / aResolution.x, 2.f / aResolution.y, 0));

        const float tanHalfAngle = std::tan(aHorizontalFOV * PI_F / 360.f);
        mPixelArea = 4.f * Sqr(tanHalfAngle) / Sqr(aResolution.x);
    }

    int RasterToIndex(const Vec2f &aPixelCoords) const
    {
        return int(std::floor(aPixelCoords.x) + std::floor(aPixelCoords.y) * mResolution.x);
    }

    Vec2f IndexToRaster(const int &aPixelIndex) const
    {
        const float y = std::floor(aPixelIndex / mResolution.x);
        const float x = float(aPixelIndex) - y * mResolution.x;
        return Vec2f(x, y);
    }

    Vec3f RasterToWorld(const Vec2f &aRasterXY) const
    {
        return mRasterToWorld.TransformPoint(Vec3f(aRasterXY.x, aRasterXY.y, 0));
    }

    Vec2f WorldToRaster(const Vec3f &aWorldPos) const
    {
        Vec3f temp = mWorldToRaster.TransformPoint(aWorldPos);
        return Vec2f(temp.x, temp.y);
    }

    // returns false when raster position is outside screen space
    bool CheckRaster(const Vec2f &aRasterPos) const
    {
        return aRasterPos.x >= 0 && aRasterPos.y >= 0 &&
            aRasterPos.x < mResolution.x && aRasterPos.y < mResolution.y;
    }


    Ray GenerateRay(const Vec2f &aRasterXY) const
    {
        const Vec3f worldRaster = RasterToWorld(aRasterXY);

        Ray res;
        res.org  = mPosition;
        res.dir  = Normalize(worldRaster - mPosition);
        res.tmin = 0;
        return res;
    }

public:
    Vec3f mPosition;
    Vec2f mResolution;
    Mat4f mRasterToWorld;
    Mat4f mWorldToRaster;

    float mPixelArea;
};

#endif //__CAMERA_HXX__