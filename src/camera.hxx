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

#ifndef __CAMERA_HXX__
#define __CAMERA_HXX__

#include <vector>
#include <cmath>
#include "math.hxx"
#include "ray.hxx"

class Camera
{
public:

    void Setup(
        const Vec3f &aPosition,
        const Vec3f &aForward,
        const Vec3f &aUp,
        const Vec2f &aResolution,
        float       aHorizontalFOV)
    {
        const Vec3f forward = Normalize(aForward);
        const Vec3f up      = Normalize(Cross(aUp, -forward));
        const Vec3f left    = Cross(-forward, up);

        mPosition   = aPosition;
        mForward    = forward;
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
        mImagePlaneDist = aResolution.x / (2.f * tanHalfAngle);
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
    Vec3f mForward;
    Vec2f mResolution;
    Mat4f mRasterToWorld;
    Mat4f mWorldToRaster;
    float mImagePlaneDist;
};

#endif //__CAMERA_HXX__