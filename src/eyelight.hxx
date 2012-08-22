/*
 * This is published under Apache 2.0
 */

#ifndef __EYELIGHT_HXX__
#define __EYELIGHT_HXX__

#include <vector>
#include <cmath>
#include "renderer.hxx"

class EyeLight : public AbstractRenderer
{
public:
    virtual void RunIteration(const Scene& aScene, Framebuffer& aoFramebuffer)
    {
        const int resX = int(aScene.mCamera.mResolution.x);
        const int resY = int(aScene.mCamera.mResolution.y);

        for(int pixID = 0; pixID < resX * resY; pixID++)
        {
            //////////////////////////////////////////////////////////////////////////
            // Generate ray
            //const int x = 16; //pixID % resX;
            //const int y = 27; //pixID / resX;
            const int x = pixID % resX;
            const int y = pixID / resX;

            const Vec2f sample(x + 0.5f, y + 0.5f);

            Ray   ray = aScene.mCamera.GenerateRay(sample);
            Isect isect;
            isect.dist = 1e36f;

            if(aScene.Intersect(ray, isect))
            {
                aoFramebuffer.AddColor(sample, Vec3f(Dot(isect.normal, -ray.dir)));
            }
        }
    }
};

#endif //__EYELIGHT_HXX__
