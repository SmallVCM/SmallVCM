/*
 * This is published under Apache 2.0
 */

#ifndef __EYELIGHT_HXX__
#define __EYELIGHT_HXX__

#include <vector>
#include <cmath>
#include <omp.h>
#include "renderer.hxx"

class EyeLight : public AbstractRenderer
{
public:
    EyeLight(Vec2f mResolution)
    {
        mIterations = 0;
        mFramebuffer.Setup(mResolution);
    }

    virtual void RunIteration(int, const Scene& aScene)
    {
        const int resX = int(aScene.mCamera.mResolution.x);
        const int resY = int(aScene.mCamera.mResolution.y);

        for(int pixID = 0; pixID < resX * resY; pixID++)
        {
            //////////////////////////////////////////////////////////////////////////
            // Generate ray
            const int x = pixID % resX;
            const int y = pixID / resX;

            const Vec2f sample(x + 0.5f, y + 0.5f);

            Ray   ray = aScene.mCamera.GenerateRay(sample);
            Isect isect;
            isect.dist = 1e36f;

            if(aScene.Intersect(ray, isect))
            {
                mFramebuffer.AddColor(sample, Vec3f(Dot(isect.normal, -ray.dir)));
            }
        }
        mIterations++;
    }

    virtual void GetFramebuffer(Framebuffer& oFramebuffer)
    {
        oFramebuffer = mFramebuffer;
        if(mIterations > 0)
            oFramebuffer.Scale(1.f / mIterations);
    }

    virtual bool WasUsed() const { return mIterations > 0; }

private:
    int         mIterations;
    Framebuffer mFramebuffer;
};

#endif //__EYELIGHT_HXX__
