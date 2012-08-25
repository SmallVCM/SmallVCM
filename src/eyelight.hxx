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
    EyeLight(const Scene& aScene) : AbstractRenderer(aScene)
    {
    }

    virtual void RunIteration(int)
    {
        const int resX = int(mScene.mCamera.mResolution.x);
        const int resY = int(mScene.mCamera.mResolution.y);

        for(int pixID = 0; pixID < resX * resY; pixID++)
        {
            //////////////////////////////////////////////////////////////////////////
            // Generate ray
            const int x = pixID % resX;
            const int y = pixID / resX;

            const Vec2f sample(x + 0.5f, y + 0.5f);

            Ray   ray = mScene.mCamera.GenerateRay(sample);
            Isect isect;
            isect.dist = 1e36f;

            if(mScene.Intersect(ray, isect))
            {
                float dotLN = Dot(isect.normal, -ray.dir);
                if(dotLN > 0)
                    mFramebuffer.AddColor(sample, Vec3f(dotLN));
                else
                    mFramebuffer.AddColor(sample, Vec3f(-dotLN, 0, 0));
            }
        }
        mIterations++;
    }
private:
};

#endif //__EYELIGHT_HXX__
